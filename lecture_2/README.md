## Ubuntu 22.04 Initial Setup Tips

### Change Ubuntu Update Server to Kakao Mirror
```bash
sudo sed -i 's/kr.archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list
```
(Tip: You should check origianl ubuntu server address. In this case, I assume that the original server address is *kr.archive.ubuntu.com*)

### Install Basic Packages
```bash
sudo apt update
sudo apt upgrade
sudo apt install build-essential
sudo apt install gcc unzip curl perl python3-pip git net-tools python3-virtualenv
sudo apt install git curl cmake-curses-gui
```

### Install Visual Studio Code
Download the `.deb` file from the official Visual Studio Code website and run:
```bash
sudo dpkg -i filename.deb
```

### Install Gemini CLI
```bash
sudo apt-get -s remove libnode-dev nodejs npm
sudo apt remove -y libnode-dev nodejs npm
sudo apt autoremove -y
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
sudo npm install -g @google/gemini-cli
```
(Tip: After installing gemini-cli, just type gemini on your terminal.)

(Tip2: You should install gemini-cli with npm rather than brew.)

### Time Synchronization (for dual-boot)
```bash
timedatectl set-local-rtc 1 --adjust-system-clock
```

