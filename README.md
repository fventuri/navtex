# NAVTEX decoder

A NAVTEX decoder library (and a simple application) based on fldigi (http://www.w1hkj.com/) excellent NAVTEX decoder by Rik van Riel, AB1KW

The source code for the original NAVTEX decoder by Rik van Riel can be found here: https://sourceforge.net/p/fldigi/fldigi/ci/master/tree/src/navtex/navtex.cxx


## How to build and install

```
git clone https://github.com/fventuri/navtex.git
cd navtex
mkdir build
cd build
cmake ..
make
sudo make install
```


## How to run the examples

```
cd build/src
./navtex_rx_from_file 11025 < ../../examples/navtex_example.res11k025
./navtex_rx_from_file 11025 < ../../examples/navtex_mondolfo.res11k025
```

To decode a NAVTEX file sampled at 48kHz:
```
./navtex_rx_from_file 48000 < navtex_filename.48k
```

To decode a NAVTEX file in .wav format, you can use 'sox' to convert it first, as follows:

```
sox <input file.wav> -b 16 -e signed -c 1 -r 11025 -t raw - | ./navtex_rx_from_file 11025
```

Same as above but using a sample rate of 48kHz instead:

```
sox <input file.wav> -b 16 -e signed -c 1 -r 48000 -t raw - | ./navtex_rx_from_file 48000
```


## Credits

- Dave Freese, W1HKJ for creating fldigi
- Rik van Riel, AB1KW for the NAVTEX decoder
- Franco Spinelli, IW2DHW for the NAVTEX example from Mondolfo, Italy


## Copyright

(C) 2021 Franco Venturi - Licensed under the GNU GPL V3 (see [LICENSE](LICENSE))
