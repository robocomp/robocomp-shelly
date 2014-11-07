Install canbus

1) 
32 and 62 bits
  sudo cp vs_can_api.h /usr/local/include/

2)
32 bits
  cp Linux/libvs_can_api.so /usr/local/lib
  
64 bits
  cp Linux64/libvs_can_api_x86-64.so /usr/local/lib64
  sudo ln -s /usr/local/lib64/libvs_can_api_x86-64.so /usr/local/lib/libvs_can_api.so

You user should be in the group dialout
sudo usermod -a -G dialout userName
