# H1 Inspire Service

Unitree H1 Inspire Hand Controller.

## Usage

```bash
# Build project
mkdir build & cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
# Terminal 1. Run h1 inspire hand service
sudo ./inspire_hand -s /dev/ttyUSB0
# Terminal 2. Run example
./h1_hand_example
```
