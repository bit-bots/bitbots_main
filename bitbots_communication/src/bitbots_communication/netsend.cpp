#include <cstdio>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <string>

#define SERVER "127.0.0.1"
#define BUFLEN 512  //Max length of buffer
#define PORT 8888   //The port on which to listen for incoming data

using namespace std;

bool udpTcp = false;
bool client = false;

void die(string s) {
    perror(s.c_str());
    exit(1);
}

int mainn(int argc, char** args) {
    for(int i = 1; i < argc; i++) {
        string arg = args[i];
        if(arg == "udp") {
            udpTcp = false;
        }

        else if(arg == "tcp") {
            udpTcp = true;
        }

        else if(arg == "server") {
            client = false;
        }

        else if(arg == "client") {
            client = true;
        }
    }
    printf("%s %s\n", (udpTcp ? "tcp" : "udp"), (client ? "client" : "server"));
    
    if(!udpTcp) {
        struct sockaddr_in si_me, si_other;

        int i, s, slen = sizeof(si_other), recv_len;
        char buf[BUFLEN];
        string message = "";

        if((s = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
            die("socket");
        } 

        if(client) {
            memset((char*) &si_other, 0, sizeof(si_other));
            si_other.sin_family = AF_INET;
            si_other.sin_port = htons(PORT);

            if(inet_aton(SERVER, &si_other.sin_addr) == 0) {
                die("inet_aton failed");
            }
        }

        else {
            memset((char*) &si_me, 0, sizeof(si_me));
            si_me.sin_family = AF_INET;
            si_me.sin_port = htons(PORT);
            si_me.sin_addr.s_addr = htonl(INADDR_ANY);

            if(bind(s, (struct sockaddr*) &si_me, sizeof(si_me)) == -1) {
                die("bind");
            }
        }

        while(true) {
            if(client) {
                printf("write something: ");
                getline(cin, message);

                if(sendto(s, message.data(), message.length(), 0, (struct sockaddr*) &si_other, slen) == -1) {
                    die("sendto");
                }

                memset(buf,'\0', BUFLEN);
                //try to receive some data, this is a blocking call
                if (recvfrom(s, buf, BUFLEN, 0, (struct sockaddr*) &si_other, (socklen_t*) &slen) == -1) {
                    die("recvfrom()");
                }

                cout << "recv: " << buf << endl;
            }

            else {
                printf("Waiting for something something...\n");
                fflush(stdout);

                memset(buf,'\0', BUFLEN);

                if((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr*) &si_other, (socklen_t*) &slen)) == -1) {
                    die("recvfrom");
                }
                //print details of the client/peer and the data received
                printf("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
                printf("Data: %s\n" , buf);
                
                for(int x = 0; x < recv_len; x++) {
                    buf[recv_len - x - 1] = buf[x];
                }

                //now reply the client with the same data
                if (sendto(s, buf, recv_len, 0, (struct sockaddr*) &si_other, slen) == -1) {
                    die("sendto()");
                }
            }
        }

        close(s);
    }

    return 0;
}