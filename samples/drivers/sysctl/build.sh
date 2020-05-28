cd app
rm -rf build
west build -b nrf5340pdk_nrf5340_cpuapp
west flash
cd ..
cd net
rm -rf build
west build -b nrf5340pdk_nrf5340_cpunet
west flash
cd ..
