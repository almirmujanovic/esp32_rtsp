#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "esp_camera.h"
#include "sensor.h"
#include "rtsp_mjpeg.h"
#include "camera_config.h"
#include "sdkconfig.h"

static const char *TAG = "rtsp_mjpeg";
static TaskHandle_t rtsp_task_handle = NULL;
static int rtsp_ctrl_sock = -1;

static uint16_t global_seq = 0;
static uint32_t global_timestamp = 0;

#define RTP_HEADER_SIZE   12
#define RTP_PAYLOAD_TYPE  26
#define RTP_SSRC          0xCAFEBABE
#define MAX_PACKET_SIZE   1400  // Increased for better efficiency
#define MAX_SEND_RETRIES  5
#define RETRY_DELAY_MS    5

// Pre-allocated packet buffer to avoid malloc/free overhead
static uint8_t packet_buffer[MAX_PACKET_SIZE];

//------------------------------------------------------------------------------
// Find end of JPEG header (SOI→SOS + length)
static int find_sos(const uint8_t *buf, size_t len)
{
    for (int i = 0; i + 3 < (int)len; i++) {
        if (buf[i] == 0xFF && buf[i+1] == 0xDA) {
            return i + 4;
        }
    }
    return 0;
}

// Build RTP header
static void build_rtp_header(uint8_t *pkt, uint16_t seq, uint32_t ts,
                             uint32_t ssrc, int marker)
{
    pkt[0] = 0x80;
    pkt[1] = (uint8_t)(RTP_PAYLOAD_TYPE | (marker ? 0x80 : 0x00));
    pkt[2] = seq >> 8;    pkt[3] = seq & 0xFF;
    pkt[4] = ts >> 24;    pkt[5] = ts >> 16;
    pkt[6] = ts >> 8;     pkt[7] = ts & 0xFF;
    pkt[8] = ssrc >> 24;  pkt[9] = ssrc >> 16;
    pkt[10] = ssrc >> 8;  pkt[11] = ssrc & 0xFF;
}

// Build JPEG payload header (8 bytes, RFC 2435)
static void build_jpeg_header(uint8_t *pkt, int offset, int type,
                              int q, int w8, int h8)
{
    pkt[0] = type;
    pkt[1] = (offset >> 16) & 0xFF;
    pkt[2] = (offset >> 8)  & 0xFF;
    pkt[3] =  offset        & 0xFF;
    pkt[4] = 0;
    pkt[5] = q;
    pkt[6] = w8;
    pkt[7] = h8;
}

// Returns 1 if DQT marker is found (only log once per session)
static int check_and_log_dqt_once(const uint8_t *buf, size_t header_len, bool *logged)
{
    for (size_t i = 0; i + 1 < header_len; ++i) {
        if (buf[i] == 0xFF && buf[i+1] == 0xDB) {
            if (!*logged) {
                ESP_LOGI(TAG, "JPEG contains DQT tables, using Q=255");
                *logged = true;
            }
            return 1;
        }
    }
    if (!*logged) {
        ESP_LOGW(TAG, "No DQT tables found, using Q=0");
        *logged = true;
    }
    return 0;
}

// Extract CSeq from an RTSP request
static bool get_cseq(const char *req, char *cseq, size_t len)
{
    const char *p = strstr(req, "CSeq:");
    if (!p) return false;
    p += 5;
    while (*p == ' ') p++;
    size_t i = 0;
    while (*p && *p!='\r' && *p!='\n' && i+1<len) {
        cseq[i++] = *p++;
    }
    cseq[i] = '\0';
    return true;
}

// Build minimal SDP
static int build_sdp(char *buf, size_t size, const char *ip)
{
    sensor_t *s = esp_camera_sensor_get();
    int width  = 320, height = 240;
    if (s) {
        framesize_t fs = s->status.framesize;
        width  = resolution[fs].width;
        height = resolution[fs].height;
    }

    return snprintf(buf, size,
        "v=0\r\n"
        "o=- 0 0 IN IP4 %s\r\n"
        "s=ESP32 MJPEG\r\n"
        "c=IN IP4 %s\r\n"
        "t=0 0\r\n"
        "m=video 0 RTP/AVP %d\r\n"
        "a=control:track1\r\n"
        "a=rtpmap:%d JPEG/90000\r\n"
        "a=framesize:%d %d-%d\r\n"
        "a=framerate:%d\r\n",
        ip, ip, RTP_PAYLOAD_TYPE, RTP_PAYLOAD_TYPE,
        RTP_PAYLOAD_TYPE, width, height,
        CONFIG_RTSP_MJPEG_DEFAULT_FPS
    );
}

// Handle complete RTSP message reception
static int recv_rtsp_message(int sock, char *buffer, size_t buffer_size, int timeout_sec)
{
    int total_received = 0;
    bool headers_complete = false;
    
    struct timeval timeout = {.tv_sec = timeout_sec, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    
    while (total_received < (int)buffer_size - 1 && !headers_complete) {
        int bytes = recv(sock, buffer + total_received, buffer_size - total_received - 1, 0);
        if (bytes <= 0) {
            if (total_received > 0) break;
            return bytes;
        }
        
        total_received += bytes;
        buffer[total_received] = '\0';
        
        if (strstr(buffer, "\r\n\r\n")) {
            headers_complete = true;
        }
        
        if (total_received > 50) {
            struct timeval short_timeout = {.tv_sec = 0, .tv_usec = 100000};
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &short_timeout, sizeof(short_timeout));
            
            char temp_buf[10];
            int extra = recv(sock, temp_buf, sizeof(temp_buf), 0);
            if (extra <= 0) {
                headers_complete = true;
            } else {
                if (total_received + extra < (int)buffer_size - 1) {
                    memcpy(buffer + total_received, temp_buf, extra);
                    total_received += extra;
                    buffer[total_received] = '\0';
                }
            }
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        }
    }
    return total_received;
}

// Optimized RTP packet sending with flow control
static bool send_rtp_packet_reliable(int sock, struct sockaddr_in *client, 
                                   const uint8_t *data, size_t len)
{
    int retry_count = 0;
    
    while (retry_count < MAX_SEND_RETRIES) {
        int sent = sendto(sock, data, len, 0, (struct sockaddr*)client, sizeof(*client));
        if (sent >= 0) {
            return true;
        }
        
        if (errno == ENOBUFS) {
            // Network buffer full - back off exponentially
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS * (1 << retry_count)));
            retry_count++;
            
            // Also yield to allow network stack to drain
            taskYIELD();
            continue;
        } else {
            // Other error - log and fail
            ESP_LOGE(TAG, "sendto failed: %d (%s)", errno, strerror(errno));
            return false;
        }
    }
    
    ESP_LOGW(TAG, "Failed to send RTP packet after %d retries", MAX_SEND_RETRIES);
    return false;
}

static void rtsp_server_task(void *pvParameters)
{
    ESP_LOGI(TAG, "RTSP server task started");

    struct sockaddr_in serv = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(CONFIG_RTSP_MJPEG_PORT)
    };

    while (1) {
        ESP_LOGI(TAG, "Creating RTSP control socket...");
        int ctrl_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (ctrl_sock < 0) {
            ESP_LOGE(TAG, "Failed to create RTSP control socket");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int opt = 1;
        setsockopt(ctrl_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        if (bind(ctrl_sock, (struct sockaddr*)&serv, sizeof(serv)) < 0) {
            ESP_LOGE(TAG, "Failed to bind RTSP control socket: %d", errno);
            close(ctrl_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (listen(ctrl_sock, 5) < 0) {
            ESP_LOGE(TAG, "Failed to listen on RTSP control socket");
            close(ctrl_sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "RTSP listening on port %d", CONFIG_RTSP_MJPEG_PORT);

        while (1) {
            struct sockaddr_in cli;
            socklen_t addrlen = sizeof(cli);
            ESP_LOGI(TAG, "Waiting for RTSP client connection...");
            
            // Log free heap before client connection
            ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
            
            int client = accept(ctrl_sock, (struct sockaddr*)&cli, &addrlen);
            if (client < 0) {
                ESP_LOGW(TAG, "accept() failed: %d", client);
                break;
            }
            ESP_LOGI(TAG, "Client connected %s", inet_ntoa(cli.sin_addr));

            // Set client socket options for better performance
            struct timeval timeout = {.tv_sec = 30, .tv_usec = 0};
            setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

            // Create RTP socket
            int rtp_sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (rtp_sock < 0) {
                ESP_LOGE(TAG, "Failed to create RTP socket");
                close(client);
                continue;
            }

            // Increase UDP send buffer
            int sndbuf = 64 * 1024;
            setsockopt(rtp_sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

            struct sockaddr_in rtp_local = {
                .sin_family = AF_INET,
                .sin_addr.s_addr = INADDR_ANY,
                .sin_port = 0
            };

            if (bind(rtp_sock, (struct sockaddr*)&rtp_local, sizeof(rtp_local)) < 0) {
                ESP_LOGE(TAG, "Failed to bind RTP socket");
                close(rtp_sock);
                close(client);
                continue;
            }

            socklen_t addr_len = sizeof(rtp_local);
            if (getsockname(rtp_sock, (struct sockaddr*)&rtp_local, &addr_len) < 0) {
                ESP_LOGE(TAG, "Failed to get RTP socket name");
                close(rtp_sock);
                close(client);
                continue;
            }
            int rtp_server_port = ntohs(rtp_local.sin_port);

            char client_ip[16];
            strcpy(client_ip, inet_ntoa(cli.sin_addr));

            struct sockaddr_in rtp_client = {0};
            rtp_client.sin_family = AF_INET;
            rtp_client.sin_addr.s_addr = cli.sin_addr.s_addr;

            bool streaming = false;
            char recv_buf[2048], resp[2048];
            int client_rtp_port = 0;
            bool dqt_logged = false;  // Track DQT logging per session

            // RTSP handshake loop
            // RTSP handshake loop - FIXED VERSION
while (!streaming) {
    ESP_LOGI(TAG, "Waiting for RTSP request...");
    
    int r = recv_rtsp_message(client, recv_buf, sizeof(recv_buf), 30);
    if (r <= 0) {
        ESP_LOGW(TAG, "RTSP recv_rtsp_message()=%d, errno=%d", r, errno);
        break; // Client disconnected or timeout
    }
    
    ESP_LOGD(TAG, "RTSP <-- (%d bytes):\n%.*s", r, r, recv_buf);

    char cseq[32] = {0};
    if (!get_cseq(recv_buf, cseq, sizeof(cseq))) {
        strcpy(cseq, "1");
    }

    // Process each RTSP method
    if (strstr(recv_buf, "OPTIONS ")) {
        ESP_LOGI(TAG, "RTSP --> OPTIONS response");
        int n = snprintf(resp, sizeof(resp),
            "RTSP/1.0 200 OK\r\n"
            "CSeq: %s\r\n"
            "Public: OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN\r\n"
            "Server: ESP32-RTSP/1.0\r\n"
            "\r\n", cseq);
        
        if (send(client, resp, n, 0) < 0) {
            ESP_LOGE(TAG, "Failed to send OPTIONS response");
            break;
        }
        // Continue to wait for next request (DESCRIBE)

    } else if (strstr(recv_buf, "DESCRIBE ")) {
        ESP_LOGI(TAG, "RTSP --> DESCRIBE response");
        char sdp[1024];
        int sdp_len = build_sdp(sdp, sizeof(sdp), client_ip);
        int n = snprintf(resp, sizeof(resp),
            "RTSP/1.0 200 OK\r\n"
            "CSeq: %s\r\n"
            "Content-Base: rtsp://%s:%d/\r\n"
            "Content-Type: application/sdp\r\n"
            "Content-Length: %d\r\n"
            "Server: ESP32-RTSP/1.0\r\n"
            "\r\n%s",
            cseq, client_ip, CONFIG_RTSP_MJPEG_PORT, sdp_len, sdp);
        
        if (send(client, resp, n, 0) < 0) {
            ESP_LOGE(TAG, "Failed to send DESCRIBE response");
            break;
        }
        // Continue to wait for next request (SETUP)

    } else if (strstr(recv_buf, "SETUP ")) {
        ESP_LOGI(TAG, "RTSP --> SETUP response");

        // Parse Transport header for client port
        int found_port = 0;
        char *transport_line = strstr(recv_buf, "Transport:");
        if (transport_line) {
            char *client_port_str = strstr(transport_line, "client_port=");
            if (client_port_str) {
                int port1 = 0;
                if (sscanf(client_port_str + strlen("client_port="), "%d", &port1) == 1) {
                    client_rtp_port = port1;
                    found_port = 1;
                    ESP_LOGI(TAG, "Parsed client RTP port: %d", client_rtp_port);
                }
            }
        }

        if (found_port && client_rtp_port > 0) {
            rtp_client.sin_port = htons(client_rtp_port);
            ESP_LOGI(TAG, "UDP Transport - Client RTP port: %d, Server RTP port: %d",
                    client_rtp_port, rtp_server_port);

            int n = snprintf(resp, sizeof(resp),
                "RTSP/1.0 200 OK\r\n"
                "CSeq: %s\r\n"
                "Transport: RTP/AVP;unicast;client_port=%d;server_port=%d\r\n"
                "Session: %08X\r\n"
                "Server: ESP32-RTSP/1.0\r\n"
                "\r\n",
                cseq, client_rtp_port, rtp_server_port, RTP_SSRC);
            
            if (send(client, resp, n, 0) < 0) {
                ESP_LOGE(TAG, "Failed to send SETUP response");
                break;
            }
            // Continue to wait for next request (PLAY)
        } else {
            ESP_LOGW(TAG, "No valid client RTP port found in SETUP request");
            int n = snprintf(resp, sizeof(resp),
                "RTSP/1.0 400 Bad Request\r\n"
                "CSeq: %s\r\n"
                "Server: ESP32-RTSP/1.0\r\n"
                "\r\n", cseq);
            
            if (send(client, resp, n, 0) < 0) {
                ESP_LOGE(TAG, "Failed to send SETUP error response");
            }
            break; // Invalid SETUP, terminate session
        }

    } else if (strstr(recv_buf, "PLAY ")) {
        ESP_LOGI(TAG, "RTSP --> PLAY response");

        // Reset counters for new session
        global_seq = 0;
        global_timestamp = 0;

        int n = snprintf(resp, sizeof(resp),
            "RTSP/1.0 200 OK\r\n"
            "CSeq: %s\r\n"
            "Session: %08X\r\n"
            "RTP-Info: url=rtsp://%s:%d/track1;seq=0;rtptime=0\r\n"
            "Server: ESP32-RTSP/1.0\r\n"
            "\r\n",
            cseq, RTP_SSRC, client_ip, CONFIG_RTSP_MJPEG_PORT);
        
        if (send(client, resp, n, 0) < 0) {
            ESP_LOGE(TAG, "Failed to send PLAY response");
            break;
        }
        
        streaming = true; // This will exit the handshake loop
        ESP_LOGI(TAG, "RTSP handshake complete, starting streaming");

    } else if (strstr(recv_buf, "TEARDOWN ")) {
        ESP_LOGI(TAG, "RTSP --> TEARDOWN response");
        int n = snprintf(resp, sizeof(resp),
            "RTSP/1.0 200 OK\r\n"
            "CSeq: %s\r\n"
            "Session: %08X\r\n"
            "Server: ESP32-RTSP/1.0\r\n"
            "\r\n", cseq, RTP_SSRC);
        
        send(client, resp, n, 0);
        break; // Client requested teardown

    } else {
        ESP_LOGW(TAG, "Unknown RTSP method in: %.50s", recv_buf);
        int n = snprintf(resp, sizeof(resp),
            "RTSP/1.0 501 Not Implemented\r\n"
            "CSeq: %s\r\n"
            "Server: ESP32-RTSP/1.0\r\n"
            "\r\n", cseq);
        
        send(client, resp, n, 0);
        // Continue loop for next request (don't break on unknown methods)
    }
} // End of handshake loop

            // Optimized streaming loop
            if (streaming && ntohs(rtp_client.sin_port) > 0) {
                ESP_LOGI(TAG, "Starting streaming to %s:%d", 
                         client_ip, ntohs(rtp_client.sin_port));
                
                TickType_t last_frame = xTaskGetTickCount();
                const TickType_t frame_period = pdMS_TO_TICKS(1000 / CONFIG_RTSP_MJPEG_DEFAULT_FPS);
                uint32_t frame_count = 0;

                while (1) {
                    // Non-blocking client check
                    int flags = fcntl(client, F_GETFL, 0);
                    fcntl(client, F_SETFL, flags | O_NONBLOCK);
                    char dummy;
                    int conn_check = recv(client, &dummy, 1, MSG_PEEK);
                    fcntl(client, F_SETFL, flags);
                    
                    if (conn_check == 0 || (conn_check < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                        ESP_LOGI(TAG, "Client disconnected during streaming");
                        break;
                    }

                    // Get camera frame
                    camera_fb_t *fb = esp_camera_fb_get();
                    if (!fb) {
                        ESP_LOGE(TAG, "Failed to get camera frame");
                        vTaskDelay(pdMS_TO_TICKS(100));
                        continue;
                    }

                    // JPEG header analysis (once per session)
                    const int jpeg_hdr_size = 8;
                    int width = fb->width;
                    int height = fb->height;
                    size_t jpeg_len = fb->len;
                    int header_len = find_sos(fb->buf, jpeg_len);
                    
                    if (header_len <= 0) {
                        ESP_LOGE(TAG, "Invalid JPEG frame");
                        esp_camera_fb_return(fb);
                        continue;
                    }

                    int q_val = check_and_log_dqt_once(fb->buf, header_len, &dqt_logged) ? 255 : 0;

                    // Check if JPEG header fits in packet
                    const int max_payload = MAX_PACKET_SIZE - RTP_HEADER_SIZE - jpeg_hdr_size;
                    if (header_len > max_payload) {
                        ESP_LOGE(TAG, "JPEG header (%d bytes) too large for packet", header_len);
                        esp_camera_fb_return(fb);
                        continue;
                    }

                    // Fragment and send
                    size_t scan_len_total = jpeg_len - header_len;
                    size_t scan_offset = 0;

                    while (scan_offset < scan_len_total) {
                        bool first_pkt = (scan_offset == 0);
                        bool last_pkt = false;

                        int chunk = max_payload;
                        if (first_pkt) chunk -= header_len;
                        if (chunk > (int)(scan_len_total - scan_offset)) {
                            chunk = scan_len_total - scan_offset;
                            last_pkt = true;
                        }

                        if (chunk <= 0) {
                            ESP_LOGE(TAG, "Invalid chunk size: %d", chunk);
                            break;
                        }

                        int pkt_size = RTP_HEADER_SIZE + jpeg_hdr_size + (first_pkt ? header_len : 0) + chunk;

                        // Use pre-allocated buffer instead of malloc
                        uint8_t *pkt = packet_buffer;

                        // Build packet
                        build_rtp_header(pkt, global_seq, global_timestamp, RTP_SSRC, last_pkt);
                        build_jpeg_header(pkt + RTP_HEADER_SIZE, scan_offset, 0, q_val, width/8, height/8);

                        int pos = RTP_HEADER_SIZE + jpeg_hdr_size;

                        if (first_pkt) {
                            memcpy(pkt + pos, fb->buf, header_len);
                            pos += header_len;
                        }

                        memcpy(pkt + pos, fb->buf + header_len + scan_offset, chunk);

                        // Send with improved reliability
                        if (!send_rtp_packet_reliable(rtp_sock, &rtp_client, pkt, pkt_size)) {
                            ESP_LOGW(TAG, "Dropping packet seq=%u", global_seq);
                        }

                        scan_offset += chunk;
                        global_seq++;
                        
                        // Yield after each packet to prevent WiFi overflow
                        taskYIELD();
                    }

                    esp_camera_fb_return(fb);
                    frame_count++;

                    // Log statistics every 100 frames
                    if (frame_count % 100 == 0) {
                        ESP_LOGI(TAG, "Sent %lu frames, heap: %lu bytes", 
                                (unsigned long)frame_count, (unsigned long)esp_get_free_heap_size());
                    }

                    // Frame rate control
                    vTaskDelayUntil(&last_frame, frame_period);
                    global_timestamp += (90000 / CONFIG_RTSP_MJPEG_DEFAULT_FPS);
                }
            }

            // Cleanup
            close(rtp_sock);
            close(client);
            ESP_LOGI(TAG, "Client session ended");
        }
        close(ctrl_sock);
        ESP_LOGI(TAG, "RTSP control socket closed, restarting server loop");
    }
}

esp_err_t rtsp_mjpeg_server_start(size_t stack_size, UBaseType_t priority)
{
    if (rtsp_task_handle) return ESP_ERR_INVALID_STATE;
    xTaskCreate(rtsp_server_task, "rtsp_server",
                stack_size, NULL, priority, &rtsp_task_handle);
    return ESP_OK;
}

esp_err_t rtsp_mjpeg_server_stop(void)
{
    if (!rtsp_task_handle) return ESP_ERR_INVALID_STATE;
    vTaskDelete(rtsp_task_handle);
    rtsp_task_handle = NULL;
    if (rtsp_ctrl_sock >= 0) {
        close(rtsp_ctrl_sock);
        rtsp_ctrl_sock = -1;
    }
    return ESP_OK;
}