CC = gcc
CFLAGS = -Wall -O2

# 定义日志相关的路径
RUNTIME_DIR = /tmp
INSTALL_ROOT = /var/lib/livelybot_logger
PROJECT_ROOT = $(INSTALL_ROOT)

# 将路径传递给编译器
CFLAGS += -DRUNTIME_DIR='"$(RUNTIME_DIR)"'
CFLAGS += -DPROJECT_ROOT='"$(PROJECT_ROOT)"'

TARGET = logger_service
SRCS = logger_service.c
OBJS = $(SRCS:.c=.o)

.PHONY: all clean install uninstall

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

install:
	install -m 755 $(TARGET) /usr/local/bin/
	install -m 644 ../scripts/livelybot-logger.service /etc/systemd/system/
	mkdir -p $(INSTALL_ROOT)
	cp -r ../* $(INSTALL_ROOT)/
	mkdir -p $(PROJECT_ROOT)/logs
	chmod 755 $(PROJECT_ROOT)/logs
	chown -R 1000:1000 $(INSTALL_ROOT)
	systemctl daemon-reload
	systemctl enable livelybot-logger.service
	systemctl start livelybot-logger.service

uninstall:
	systemctl stop livelybot-logger.service
	systemctl disable livelybot-logger.service
	rm -f /etc/systemd/system/livelybot-logger.service
	rm -f /usr/local/bin/$(TARGET)
	rm -rf $(INSTALL_ROOT)

clean:
	rm -f $(OBJS) $(TARGET) 