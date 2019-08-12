#include<iostream>
#include<unistd.h>
#include<boost/filesystem.hpp>
#include<stdio.h>
#include "httplib.h"

#define SHARD_PATH "SHARD"
#define LOG(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)

using namespace httplib;
namespace bf = boost::filesystem;

class P2PServer
{
  private:
    Server _server;
  private:
    static void GetHostPair(const Request& req,Response& rsp)
    {
      rsp.status =200;
      return;
    }
    static void GetFileList(const Request& req,Response& rsp)
    {
      bf::directory_iterator it_begin(SHARD_PATH);
      bf::directory_iterator it_end;
      std::stringstream body;
     // body<<"<html><body>";
      for(;it_begin!=it_end;it_begin++){
        if(bf::is_directory(it_begin->status())){
            continue;
        }
        std::string name = it_begin->path().filename().string();
        rsp.body+=name+"\n";
       // body << "<h4><a href='/list/"<< name <<"'>";
       // body << name;
       // body <<"</a></h4>";
       //rsp.body+=name;
      }
     // body<<"</body></html>";
     // rsp.body = body.str();
      rsp.set_header("Content-Type","text/html");//渲染
      rsp.status = 200;
    }

    static void GetFileData(const Request& req,Response& rsp)
    {
      //a.txt->Dowanload/a.txt
      bf::path path(req.path); //生成一个path对象
      std::stringstream name;
      name<< SHARD_PATH << "/" << path.filename().string();

      if(!bf::exists(name.str())){
          rsp.status = 404;
          return;
      }

      if(bf::is_directory(name.str())){
          rsp.status = 403;
          return;
      }

      int64_t fsize = bf::file_size(name.str());
      if(req.method == "HEAD"){
        rsp.status = 200;
        std::string len = std::to_string(fsize);
        rsp.set_header("Content-Length", len.c_str());
        return;
      }
      else{
          if(!req.has_header("Range")){
              rsp.status = 400;
              return;
          }
          std::string range_val;
          range_val = req.get_header_value("Range");
          int64_t start, rlen;
          
          bool ret = RangeParse(range_val, start, rlen);
          if(ret == false){
            rsp.status = 400;
            return;
          }

          std::cerr << "body.resize: " << rlen << "\n";
          rsp.body.resize(rlen);
          std::ifstream file(name.str(),std::ios::binary);
          if(!file.is_open())
          {
              std::cerr<<"open file"<<name.str()<<"failed\n";
              rsp.status = 404;
              return;
          }
          file.seekg(start, std::ios::beg);
          file.read(&rsp.body[0],rlen); //写到body里了
          if(!file.good()){
              std::cerr<<"read file"<<name.str()<<"body error\n";
              rsp.status = 500;
              return;
          }
          file.close();
          rsp.status = 206;
          rsp.set_header("Content-Type","application/octet-stream");//二进制流
          std::cerr << "file range: " << range_val;
          std::cerr << "download success\n";
      }
    }

    static bool RangeParse(std::string &range_val, int64_t &start, \
                    int64_t &len){
        size_t pos1 = range_val.find("=");
        size_t pos2 = range_val.find("-");
        if(pos1 == std::string::npos ||
           pos2 == std::string::npos){
            std::cerr << "range " << range_val << " format error\n";
            return false;
        }
        int64_t end;
        std::string rstart;
        std::string rend;
        rstart = range_val.substr(pos1 + 1, pos2 - pos1 - 1);
        rend = range_val.substr(pos2 + 1);

        std::stringstream tmp;
        tmp << rstart;
        tmp >> start;
        tmp.clear();
        tmp << rend;
        tmp >> end;
        len = end - start + 1;
        return true;
    }

    public:
    P2PServer(){
      if(!bf::exists(SHARD_PATH)){
          bf::create_directory(SHARD_PATH);
      }
    }
    bool Start(uint16_t port){
      _server.Get("/hostpair",GetHostPair);
      _server.Get("/list",GetFileList);
      _server.Get("/list/(.*)",GetFileData);
      _server.listen("0.0.0.0",port);
    }
};

int main()
{
  P2PServer srv;
  srv.Start(9000);
  return 0;
}
