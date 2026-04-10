#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <signal.h>
#include <errno.h>
#include <dirent.h>
#include <pwd.h>
#include <sys/time.h>
#include <ctype.h>
#include <libgen.h>
#include <limits.h>

// 配置宏，可以通过编译时定义覆盖
#ifndef RUNTIME_DIR
#define RUNTIME_DIR "/tmp"
#endif

#ifndef PROJECT_ROOT
#define PROJECT_ROOT "."
#endif

#define SOCKET_PATH RUNTIME_DIR "/logger_service.sock"
#define LOGS_DIR PROJECT_ROOT "/logs"
#define MAX_LOG_AGE_DAYS 30 // 日志文件保存时间
#define BUFFER_SIZE 4096    // 缓冲区大小
#define ROS_CHECK_INTERVAL 3 // 检查ROS连接的间隔（秒）

#define MSG_TYPE_HEARTBEAT "HEARTBEAT"
#define MSG_TYPE_STATUS "STATUS:"
#define MSG_TYPE_OPERATION "OPERATION:"

static int running = 1;
static int server_fd;
static struct timeval last_msg_time; // 最后一次收到消息的时间
static int ros_connected = 0;        // ROS连接状态

void print_user_info()
{
    uid_t uid = getuid();
    gid_t gid = getgid();
    struct passwd *pw = getpwuid(uid);
    printf("Running as user: %s (uid=%d, gid=%d)\n", pw ? pw->pw_name : "unknown", uid, gid);
}

void write_status_to_log(const char *message, const char *timestamp)
{
    char filename[64];
    char filepath[256];
    FILE *fp;
    time_t now;
    struct tm *timeinfo;
    int ros_node_header_printed = 0; // 用于跟踪是否已经打印了ROS节点标题
    int ros_node_count = 0;          // 用于计数处理的节点数量

    time(&now);
    timeinfo = localtime(&now);
    strftime(filename, sizeof(filename), "%Y-%m-%d.log", timeinfo);
    snprintf(filepath, sizeof(filepath), "%s/%s", LOGS_DIR, filename);

    fp = fopen(filepath, "a");
    if (fp == NULL)
    {
        printf("Cannot open log file %s: %s\n", filepath, strerror(errno));
        return;
    }

    // 写入时间戳和标题 - 使用示例格式
    fprintf(fp, "%s System Status:\n", timestamp);

    // 处理状态数据，按模块分类处理
    const char *start = message + strlen(MSG_TYPE_STATUS);
    char *status_copy = strdup(start);
    char *saveptr1, *saveptr2, *saveptr3;
    char *module = strtok_r(status_copy, ";", &saveptr1);

    while (module != NULL)
    {
        // 去除前后空格
        while (*module && isspace((unsigned char)*module))
            module++;

        if (strncmp(module, "MOTOR", 5) == 0)
        {
            fprintf(fp, "                        [Motors]\n");
            // 处理所有电机数据
            module = strtok_r(NULL, ";", &saveptr1); // 获取第一个电机数据
            int motor_num = 1;
            while (module != NULL && module[0] == 'M')
            {
                char *motor_copy = strdup(module);
                char *data = strtok_r(motor_copy, ",", &saveptr2);
                fprintf(fp, "                            M %d:", motor_num++);

                while (data != NULL)
                {
                    if (strncmp(data, "pos:", 4) == 0)
                        fprintf(fp, "P: %s", data + 4);
                    else if (strncmp(data, "vel:", 4) == 0)
                        fprintf(fp, " , V: %s", data + 4);
                    else if (strncmp(data, "tor:", 4) == 0)
                        fprintf(fp, " , T: %s", data + 4);
                    data = strtok_r(NULL, ",", &saveptr2);
                }
                fprintf(fp, "\n");
                free(motor_copy);
                module = strtok_r(NULL, ";", &saveptr1);
            }
            continue; // 继续处理下一个模块
        }
        else if (strncmp(module, "IMU", 3) == 0)
        {
            fprintf(fp, "                        [IMU]\n");
            char *imu_copy = strdup(module);
            char *data = strtok_r(imu_copy, ",", &saveptr2);
            data = strtok_r(NULL, ",", &saveptr2); // 跳过"IMU"标识

            while (data != NULL)
            {
                if (strncmp(data, "acc:", 4) == 0)
                {
                    char *acc_copy = strdup(data + 4);
                    char *val = strtok_r(acc_copy, "/", &saveptr3);
                    fprintf(fp, "                            Acceleration: X: %s", val);
                    val = strtok_r(NULL, "/", &saveptr3);
                    fprintf(fp, " , Y: %s", val);
                    val = strtok_r(NULL, "/", &saveptr3);
                    fprintf(fp, " , Z: %s\n", val);
                    free(acc_copy);
                }
                else if (strncmp(data, "gyro:", 5) == 0)
                {
                    char *gyro_copy = strdup(data + 5);
                    char *val = strtok_r(gyro_copy, "/", &saveptr3);
                    fprintf(fp, "                            Angular Velocity: X: %s", val);
                    val = strtok_r(NULL, "/", &saveptr3);
                    fprintf(fp, " , Y: %s", val);
                    val = strtok_r(NULL, "/", &saveptr3);
                    fprintf(fp, " , Z: %s\n", val);
                    free(gyro_copy);
                }
                else if (strncmp(data, "euler:", 6) == 0)
                {
                    char *euler_copy = strdup(data + 6);
                    char *val = strtok_r(euler_copy, "/", &saveptr3);
                    fprintf(fp, "                            Euler Angles: Roll: %s", val);
                    val = strtok_r(NULL, "/", &saveptr3);
                    fprintf(fp, " , Pitch: %s", val);
                    val = strtok_r(NULL, "/", &saveptr3);
                    fprintf(fp, " , Yaw: %s\n", val);
                    free(euler_copy);
                }
                data = strtok_r(NULL, ",", &saveptr2);
            }
            free(imu_copy);
        }
        else if (strncmp(module, "POWER", 5) == 0)
        {
            fprintf(fp, "                        [Power Board]\n");
            char *power_copy = strdup(module);
            char *data = strtok_r(power_copy, ",", &saveptr2);
            data = strtok_r(NULL, ",", &saveptr2); // 跳过"POWER"标识

            fprintf(fp, "                            ");
            int first = 1;
            while (data != NULL)
            {
                if (strncmp(data, "sys:", 4) == 0)
                {
                    fprintf(fp, "System Power: %s", strcmp(data + 4, "1") == 0 ? "ON" : "OFF");
                    first = 0;
                }
                else if (strncmp(data, "motor:", 6) == 0)
                {
                    if (!first)
                        fprintf(fp, " , ");
                    fprintf(fp, "Motor Power: %s", strcmp(data + 6, "1") == 0 ? "ON" : "OFF");
                }
                data = strtok_r(NULL, ",", &saveptr2);
            }
            fprintf(fp, "\n");
            free(power_copy);
        }
        else if (strncmp(module, "BATTERY", 7) == 0)
        {
            fprintf(fp, "                        [Battery]\n");
            char *battery_copy = strdup(module);
            char *data = strtok_r(battery_copy, ",", &saveptr2);
            data = strtok_r(NULL, ",", &saveptr2); // 跳过"BATTERY"标识

            fprintf(fp, "                            ");
            int first = 1;
            while (data != NULL)
            {
                if (!first)
                    fprintf(fp, " , ");
                if (strncmp(data, "volt:", 5) == 0)
                    fprintf(fp, "Voltage: %sV", data + 5);
                else if (strncmp(data, "curr:", 5) == 0)
                    fprintf(fp, "Current: %sA", data + 5);
                else if (strncmp(data, "temp:", 5) == 0)
                    fprintf(fp, "Temperature: %s°C", data + 5);
                else if (strncmp(data, "remain:", 7) == 0)
                    fprintf(fp, "Remaining: %s%%", data + 7);
                first = 0;
                data = strtok_r(NULL, ",", &saveptr2);
            }
            fprintf(fp, "\n");
            free(battery_copy);
        }
        else if (strncmp(module, "SYSTEM_RESOURCES", 16) == 0)
        {
            fprintf(fp, "                        [System Resources]\n");
            char *resources_copy = strdup(module);
            char *data = strtok_r(resources_copy, ",", &saveptr2);
            data = strtok_r(NULL, ",", &saveptr2); // 跳过"SYSTEM_RESOURCES"标识

            fprintf(fp, "                            ");
            int first = 1;
            while (data != NULL)
            {
                if (!first)
                    fprintf(fp, " , ");
                if (strncmp(data, "cpu:", 4) == 0)
                    fprintf(fp, "CPU : %s", data + 4);
                else if (strncmp(data, "mem:", 4) == 0)
                    fprintf(fp, "Memory : %s", data + 4);
                else if (strncmp(data, "disk:", 5) == 0)
                    fprintf(fp, "Disk : %s", data + 5);
                else if (strncmp(data, "cpu_temp:", 9) == 0)
                    fprintf(fp, "CPU Temperature: %s", data + 9);
                else if (strncmp(data, "data:", 5) == 0)
                    fprintf(fp, "Resources: %s", data + 5);
                first = 0;
                data = strtok_r(NULL, ",", &saveptr2);
            }
            fprintf(fp, "\n");
            free(resources_copy);
        }
        else if (strncmp(module, "ROS_NODE", 8) == 0)
        {
            // 仅在第一次遇到ROS_NODE时打印标题
            if (!ros_node_header_printed)
            {
                fprintf(fp, "                        [ROS Nodes]\n");
                ros_node_header_printed = 1;
            }

            ros_node_count++; // 增加节点计数

            char *node_copy = strdup(module);
            char *data = strtok_r(node_copy, ",", &saveptr2);
            data = strtok_r(NULL, ",", &saveptr2); // 跳过"ROS_NODE"标识

            char node_name[128] = "";
            char node_status[32] = "";
            char node_cpu[32] = "";
            char node_memory[32] = "";

            // 解析节点信息
            while (data != NULL)
            {
                if (strncmp(data, "node:", 5) == 0)
                    snprintf(node_name, sizeof(node_name), "%s", data + 5);
                else if (strncmp(data, "status:", 7) == 0)
                    snprintf(node_status, sizeof(node_status), "%s", data + 7);
                else if (strncmp(data, "cpu:", 4) == 0)
                    snprintf(node_cpu, sizeof(node_cpu), "%s", data + 4);
                else if (strncmp(data, "memory:", 7) == 0)
                    snprintf(node_memory, sizeof(node_memory), "%s", data + 7);

                data = strtok_r(NULL, ",", &saveptr2);
            }

            // 打印节点信息，严格按照示例格式
            fprintf(fp, "                            Node: %-25s Status: %-8s CPU: %-8s Memory: %-8s\n",
                    node_name, node_status, node_cpu, node_memory);

            free(node_copy);
        }

        module = strtok_r(NULL, ";", &saveptr1);
    }

    printf("Processed status message: found %d ROS nodes\n", ros_node_count);
    free(status_copy);
    fclose(fp);
}

void write_to_log(const char *message)
{
    time_t now;
    struct tm *timeinfo;
    char filename[64];
    char filepath[256];
    char timestamp[32];
    FILE *fp;

    time(&now);
    timeinfo = localtime(&now);

    strftime(filename, sizeof(filename), "%Y-%m-%d.log", timeinfo);
    strftime(timestamp, sizeof(timestamp), "[%Y-%m-%d %H:%M:%S]", timeinfo);

    // 简化处理逻辑，移除分块节点处理
    // 根据消息类型选择不同的处理方式
    if (strncmp(message, MSG_TYPE_STATUS, strlen(MSG_TYPE_STATUS)) == 0)
    {
        // 直接处理状态信息
        write_status_to_log(message, timestamp);
        return;
    }
    else if (strncmp(message, MSG_TYPE_OPERATION, strlen(MSG_TYPE_OPERATION)) == 0)
    {
        // 操作消息，使用原有格式，但去掉前缀
        message += strlen(MSG_TYPE_OPERATION);
    }
    else if (strcmp(message, "ROS is connected") == 0 ||
             strcmp(message, "ROS is not connected") == 0 ||
             strcmp(message, "Logger service started") == 0)
    {
        // 系统消息，使用原有格式
    }
    else
    {
        // 未知类型消息，添加警告
        printf("Warning: Unknown message type received: %s\n", message);
        return;
    }

    snprintf(filepath, sizeof(filepath), "%s/%s", LOGS_DIR, filename);
    fp = fopen(filepath, "a");
    if (fp == NULL)
    {
        printf("Cannot open log file %s: %s\n", filepath, strerror(errno));
        return;
    }
    fprintf(fp, "%s %s\n", timestamp, message);
    fclose(fp);
}

void check_ros_connection()
{
    struct timeval now;
    gettimeofday(&now, NULL);

    double time_diff = (double)(now.tv_sec - last_msg_time.tv_sec) +
                       (double)(now.tv_usec - last_msg_time.tv_usec) / 1000000.0;

    if (ros_connected && time_diff >= ROS_CHECK_INTERVAL)
    {
        write_to_log("ROS is not connected");
        ros_connected = 0;
    }
}

void signal_handler(int signum)
{
    if (signum == SIGTERM || signum == SIGINT)
    {
        running = 0;
        close(server_fd);
        unlink(SOCKET_PATH);
        exit(0);
    }
}

int main()
{
    struct sockaddr_un addr;
    char buffer[BUFFER_SIZE];
    struct timeval timeout;
    fd_set readfds;

    print_user_info();
    mkdir(LOGS_DIR, 0777);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    server_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (server_fd == -1)
    {
        perror("socket error");
        exit(1);
    }

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

    unlink(SOCKET_PATH);
    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
    {
        perror("bind error");
        exit(1);
    }

    printf("Logger service started, logs will be written to %s\n", LOGS_DIR);
    printf("Buffer size set to %d bytes\n", BUFFER_SIZE);
    write_to_log("Logger service started");

    gettimeofday(&last_msg_time, NULL);

    while (running)
    {
        FD_ZERO(&readfds);
        FD_SET(server_fd, &readfds);

        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ret = select(server_fd + 1, &readfds, NULL, NULL, &timeout);

        if (ret > 0)
        {
            ssize_t bytes_received = recv(server_fd, buffer, BUFFER_SIZE - 1, 0);
            if (bytes_received > 0)
            {
                buffer[bytes_received] = '\0';
                printf("Received message (%zd bytes): %.60s...\n", bytes_received, buffer);

                if (!ros_connected)
                {
                    write_to_log("ROS is connected");
                    ros_connected = 1;
                }

                if (strcmp(buffer, MSG_TYPE_HEARTBEAT) != 0)
                {
                    write_to_log(buffer);
                }

                gettimeofday(&last_msg_time, NULL);
            }
        }
        else if (ret == 0)
        {
            check_ros_connection();
        }
        else
        {
            printf("Select error: %s\n", strerror(errno));
        }
    }
    return 0;
}
