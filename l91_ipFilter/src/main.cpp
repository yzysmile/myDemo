# include <iostream>
# include <string>
# include <map>
# include <vector>
# include <unordered_map>

std::map<int, std::string> initializePeerUri()
{
  std::map<int, std::string> protocol_map;
  protocol_map.insert(std::make_pair(1 , "192.168.31.144:60035"));
  protocol_map.insert(std::make_pair(2 , "192.168.31.144:50009"));
  protocol_map.insert(std::make_pair(2 , "192.168.31.131:78361"));
  
  return protocol_map;
}

int main()
{
  std::map<int, std::string> id_peerUris = initializePeerUri();

  std::unordered_map<std::string, std::string> ip_peerUris;
  std::vector<std::string> peerUris;

  for(auto& id_peerUri : id_peerUris){
    auto peerUri = id_peerUri.second;
    auto ip = id_peerUri.second.substr(0, id_peerUri.second.length() - 6);

    if(ip_peerUris.find(ip) ==  ip_peerUris.end()){
       ip_peerUris.insert(std::make_pair(ip, peerUri));
    }else{
       ip_peerUris.erase(ip);
       std::cout << "new:" << peerUri << std::endl;
       ip_peerUris.insert(std::make_pair(ip, peerUri));
    }
  }

  for(auto peerUri:ip_peerUris){
    std::cout << "return peer is:" << peerUri.second << std::endl;
    peerUris.emplace_back(peerUri.second);
  }
}