# Ghid de pornire — Recon-Platform-R2

1. Clonează proiectul pe un Raspberry Pi care are drept OS ubuntu server 22.04 LTS:
   `git clone -b handheld https://github.com/ValentinGabrie/Recon-Platform-R2.git`
2. Pe Raspberry Pi, instalează dependințele:
   `cd Recon-Platform-R2/roomba_ws`
   `bash environment.sh`
3. Build workspace ROS2 (tot pe Pi):
   `source /opt/ros/jazzy/setup.bash`
   `colcon build --symlink-install`
4. Flash ESP32 de pe PC (VS Code + PlatformIO):

   - deschide folderul `firmware/esp32` în VS Code
   - conectează ESP32 prin USB (apare ca `COMx`)
   - apasă **Build** , apoi **Upload**
5. Pornește sistemul pe Pi:
   `cd roomba_ws`
   `bash setup.sh sensor-test`
6. Conecteazate la Raspbery Pi dupa un minut de la pornire:

   1. SSID: Recon
   2. Pasword: recon123
7. Deschide interfața în browser la adresa IP 10.0.0.1
