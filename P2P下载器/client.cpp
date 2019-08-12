#include <iostream>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <fstream>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "httplib.h"
#include <thread>

#define RANGE_SIZE (1024*1024*10)   // 10M
using namespace httplib;
namespace bf = boost::filesystem;

class P2PClient{

private:
    uint16_t _srv_port;
    int _host_idx;
    std::vector<std::string> _online_list;
    std::vector<std::string> _file_list;
private:
    bool GetAllHost(std::vector<std::string> &list){

        struct sockaddr_in *ip = NULL;
        struct sockaddr_in *mask = NULL;
        struct ifaddrs *addrs = NULL;
        getifaddrs(&addrs);
        for(; addrs != NULL; addrs = addrs->ifa_next){
            ip = (struct sockaddr_in*)addrs->ifa_addr;
            mask = (struct sockaddr_in*)addrs->ifa_netmask;
            if(ip->sin_family != AF_INET){
                continue;
            }

            if(ip->sin_addr.s_addr == inet_addr("127.0.0.1")){
                continue;
            }

            uint32_t net, host;
            net = ntohl(ip->sin_addr.s_addr & mask->sin_addr.s_addr);
            host = ntohl(~mask->sin_addr.s_addr);
            for(int i = 2; i < host-1; i++){
                struct in_addr ip;
                ip.s_addr = htonl(net+i);
                list.push_back(inet_ntoa(ip));
            }
        }
        freeifaddrs(addrs);


        return true;
    }

    void HostPair(std::string &i){
        Client client(i.c_str(), _srv_port);
        auto req = client.Get("/hostpair");
        if(req && req->status == 200){
            std::cerr << "host " << i << " pair success\n";
            _online_list.push_back(i);
        }
        std::cerr << "host " << i << " pair failed\n";

        return;
    }

    bool GetOnlineHost(std::vector<std::string> &list){
        _online_list.clear();
        std::vector<std::thread> thr_list(list.size());
        for(int i = 0; i < list.size(); i++){
            std::thread thr(&P2PClient::HostPair, this, std::ref(list[i]));
            thr_list[i] = std::move(thr);
        }

        for(int i = 0; i < thr_list.size(); i++){
            thr_list[i].join();
        }
        return true;
    }

    bool ShowOnlineHost(){
        for(int i = 0; i < _online_list.size(); i++){
            std::cout << i << ". " << _online_list[i] << "\n";
        }
        std::cout << "please choose:";
        fflush(stdout);
        std::cin >> _host_idx;
        if(_host_idx < 0 || _host_idx > _online_list.size()){
            std::cerr << "choose error\n";
            return false;
        }
        return true;
    }

    bool GetFileList(){
        Client client(_online_list[_host_idx].c_str(), _srv_port);
        auto req = client.Get("/list");
        if(req && req->status == 200){
            //std::vector<std::string> list;
            boost::split(_file_list, req->body, boost::is_any_of("\n"));
        }
        return true;
    }

    bool ShowFileList(std::string &name){
        for(int i = 0; i < _file_list.size(); i++){
            std::cout << i << ". " << _file_list[i] << "\n";
        }
        std::cout << "please choose:";
        fflush(stdout);
        int file_idx;
        std::cin >> file_idx;
        if(file_idx < 0 || file_idx > _file_list.size()){
            std::cerr << "choose error\n";
            return false;
        }
        name = _file_list[file_idx];
        return true;
    }

    void RangeDownLoad(std::string host, std::string name, 
                       int64_t start, int64_t end, int *res){
        std::string uri = "/list/" + name;
        std::string realpath = "Download/" + name;
        std::stringstream range_val;
        range_val << "bytes=" << start << "-" << end;
        
        std::cerr << "download range: " << range_val.str() << "\n"; 
        *res = 0;
        Client client(host.c_str(), _srv_port);
        Headers header;
        header.insert(std::make_pair("Range", range_val.str().c_str()));
        auto req = client.Get(uri.c_str(), header);
        if(req && req->status == 206){
            int fd = open(realpath.c_str(), O_CREAT | O_WRONLY, 0664);
            if(fd < 0){
                std::cerr << "file " << realpath << " open error\n";
                return;
            }
            lseek(fd, start, SEEK_SET);
            int ret = write(fd, &req->body[0], req->body.size());
            if(ret < 0){
                std::cerr << "file " << realpath << "write error\n";
                close(fd);
                return;
            }
            close(fd);
            *res = 1;
            std::cerr << "file " << realpath << " download range:";
            std::cerr << range_val.str() << " success\n";
        }
        return;
        
    }

    int64_t GetFileSize(std::string &host, std::string &name){
        int64_t fsize = -1;
        std::string path = "/list/" + name;
        Client client(host.c_str(), _srv_port);
        auto req = client.Head(path.c_str());
        if(req && req->status == 200){
            if(!req->has_header("Content-Length")){
                return -1;
            }
            std::string len = req->get_header_value("Content-Length");
            std::stringstream tmp;
            tmp << len;
            tmp >> fsize;
        }
        return fsize;
    }

    bool DownLoadFile(std::string &name){
        // GET /list/filename
        std::string host = _online_list[_host_idx];
        int64_t fsize = GetFileSize(host, name);    
        if(fsize < 0){
            std::cerr << "download file " << name << "failed\n";
            return false;
        }

        int count = fsize / RANGE_SIZE;
        std::cout << "range count: " << count << "\n";
        std::vector<boost::thread> thr_list(count + 1);
        std::vector<int> res_list(count + 1);
        for(int64_t i = 0; i <= count; i++){
            int64_t start, end, len;
            start = i * RANGE_SIZE;
            end = (i + 1) * RANGE_SIZE - 1;
            if(i == count){
                if(fsize % RANGE_SIZE == 0){
                    break;
                }
                end = fsize - 1;
            }
            std::cerr << "range: " << start << "-" << end <<"\n";
            len = end - start + 1;
            
            int *res = &res_list[i];
            boost::thread thr(&P2PClient::RangeDownLoad, this, host, name, start, end, res);
            thr_list[i] = std::move(thr);
        }

        bool ret = true;
        for(int i = 0; i <= count; i++){
            if(i == count && fsize % RANGE_SIZE == 0){
                break;
            }
            thr_list[i].join();
            if(res_list[i] == 0){
                ret = false;
            }
        }

        if(ret == true){
            std::cerr << "download file " << name << " success\n";
        }
        else{
            std::cerr << "download file " << name << " failed\n";
            return false;
        }
        return true;
    }

    int DoFace(){
        std::cout << "0. 退出\n";
        std::cout << "1. 搜索附近主机\n";
        std::cout << "2. 显示在线主机\n";
        std::cout << "3. 显示文件列表\n";
        int choose = 0;
        std::cout << "please choose:";
        fflush(stdout);
        std::cin >> choose;
        return choose;
    }
public:
    P2PClient(uint16_t port):_srv_port(port){}
    bool Start(){
        while(1){
            int choose = DoFace();
            std::string filename;
            std::vector<std::string> list;
            switch(choose){
            case 1:
                GetAllHost(list);
                GetOnlineHost(list);
                break;
            case 2:
                if(ShowOnlineHost() == false){
                    break;
                }
                GetFileList();
                break;
            case 3:
                if(ShowFileList(filename) == false){
                    break;
                }
                DownLoadFile(filename);
                break;
            case 0:
                exit(0);
            default:
                break;
            }
        }
    }

};

int main(){

    P2PClient cli(9000);
    cli.Start();
    return 0;
}
