
```cross build --target armv7-unknown-linux-gnueabihf```  
```scp target/armv7-unknown-linux-gnueabihf/debug/all_in_one_enviro_mini dawid@192.168.1.27://home/dawid/Workspace```




```
/usr/bin/docker build --label 'org.cross-rs.for-cross-target=armv7-unknown-linux-gnueabihf' --label 'org.cross-rs.workspace_root=/home/dawid/Workspace/pi/enviroplus-rust/all_in_one_enviro_mini' --tag cross-custom-all_in_one_enviro_mini:armv7-unknown-linux-gnueabihf-92fc0-pre-build --build-arg 'CROSS_CMD=dpkg --add-architecture armv7 && apt-get update && apt-get install --assume-yes wget' --build-arg 'CROSS_DEB_ARCH=armhf' --file /home/dawid/Workspace/pi/enviroplus-rust/all_in_one_enviro_mini/target/armv7-unknown-linux-gnueabihf/Dockerfile.armv7-unknown-linux-gnueabihf-custom /home/dawid/Workspace/pi/enviroplus-rust/all_in_one_enviro_mini



/usr/bin/docker run -e 'RUST_BACKTRACE=1' -e 'PKG_CONFIG_ALLOW_CROSS=1' -e 'XARGO_HOME=/xargo' -e 'CARGO_HOME=/cargo' -e 'CARGO_TARGET_DIR=/target' -e 'CROSS_RUNNER=' -e TERM -e 'USER=dawid' -v /home/dawid/Workspace/pi/enviroplus-rust/embedded-canvas:/home/dawid/Workspace/pi/enviroplus-rust/embedded-canvas -v /home/dawid/Workspace/pi/enviroplus-rust/ltr-559:/home/dawid/Workspace/pi/enviroplus-rust/ltr-559 -v /home/dawid/Workspace/pi/enviroplus-rust/rust-portaudio:/home/dawid/Workspace/pi/enviroplus-rust/rust-portaudio -v 
/home/dawid/Workspace/pi/enviroplus-rust/rust-portaudio/rust-portaudio-sys:/home/dawid/Workspace/pi/enviroplus-rust/rust-portaudio/rust-portaudio-sys -v /home/dawid/Workspace/pi/enviroplus-rust/st7735-lcd-rs:/home/dawid/Workspace/pi/enviroplus-rust/st7735-lcd-rs -v /home/dawid/.xargo:/xargo:z -v /home/dawid/.cargo:/cargo:z -v /cargo/bin -v /home/dawid/Workspace/pi/enviroplus-rust/all_in_one_enviro_mini:/home/dawid/Workspace/pi/enviroplus-rust/all_in_one_enviro_mini:z -v /home/dawid/.rustup/toolchains/stable-x86_64-unknown-linux-gnu:/rust:z,ro -v /home/dawid/Workspace/pi/enviroplus-rust/all_in_one_enviro_mini/target:/target:z -w /home/dawid/Workspace/pi/enviroplus-rust/all_in_one_enviro_mini -i -t cross-custom-all_in_one_enviro_mini:armv7-unknown-linux-gnueabihf-92fc0-pre-build sh -c 'PATH=$PATH:/rust/bin cargo -v build --target armv7-unknown-linux-gnueabihf'



/usr/bin/docker run --userns host -e 'RUST_BACKTRACE=1' -e 'PKG_CONFIG_ALLOW_CROSS=1' -e 'XARGO_HOME=/xargo' -e 'CARGO_HOME=/cargo' -e 'CARGO_TARGET_DIR=/target' -e 'CROSS_RUNNER=' -e TERM -e 'USER=dawid' -v /home/dawid/Workspace/pi/enviroplus-rust/rust-portaudio/rust-portaudio-sys:/home/dawid/Workspace/pi/enviroplus-rust/rust-portaudio/rust-portaudio-sys --user 1000:1000 -v /home/dawid/.xargo:/xargo:z -v /home/dawid/.cargo:/cargo:z -v /cargo/bin -v /home/dawid/Workspace/pi/enviroplus-rust/rust-portaudio:/home/dawid/Workspace/pi/enviroplus-rust/rust-portaudio:z -v /home/dawid/.rustup/toolchains/stable-x86_64-unknown-linux-gnu:/rust:z,ro -v /home/dawid/Workspace/pi/enviroplus-rust/rust-portaudio/target:/target:z -w /home/dawid/Workspace/pi/enviroplus-rust/rust-portaudio -i -t cross-custom-rust-portaudio:armv7-unknown-linux-gnueabihf-b3ce6-pre-build sh -c 'PATH=$PATH:/rust/bin cargo -v build --target armv7-unknown-linux-gnueabihf'


```


## Crossbuilding

```shell
apt-get install gcc-arm-linux-gnueabihf make autoconf automake git
export CC=arm-linux-gnueabihf-gcc
export AR=/usr/bin/arm-linux-gnueabihf-gcc-ar
ln -s /usr/bin/arm-linux-gnueabihf-gcc-ar /usr/bin/ar
ln -s /usr/arm-linux-gnueabihf/bin/objdump /usr/bin/objdump
git clone https://github.com/PortAudio/portaudio.git
./configure --disable-shared --enable-static  --with-pic --host=arm-linux-gnueabihf

rustup toolchain install stable-arm-unknown-linux-gnueabihf
rustup target add arm-unknown-linux-gnueabihf
apt install build-essential wget
git clone https://github.com/RustAudio/rust-portaudio.git


./configure --disable-shared --enable-static --with-alsa --with-pic --host arm-linux-gnueabihf
export LIBS='-lpthread -ldl'

	    //	    .args(&["--enable-shared"]) // Only build static lib
	    .env("LIBS","-lpthread -ldl")
	    .args(&["--host", "arm-linux-gnueabihf"]) // cross compile
//	    .args(&["--with-alsa"]) // Build position-independent code (required by Rust)
```
