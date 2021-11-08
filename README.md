# NAVTEX decoder

A NAVTEX decoder library (and a simple application) based on fldigi (http://www.w1hkj.com/) excellent NAVTEX decoder by Rik van Riel, AB1KW

The source code for the original NAVTEX decoder by Rik van Riel can be found here: https://sourceforge.net/p/fldigi/fldigi/ci/master/tree/src/navtex/navtex.cxx


## How to build

```
git clone https://github.com/fventuri/navtex.git
cd navtex
mkdir build
cd build
cmake ..
make
```


## How to run the examples

```
cd build/src
./navtex_rx_from_file < ../../examples/navtex_example.res11k025
./navtex_rx_from_file < ../../examples/navtex_mondolfo.res11k025
```


## Credits

- Dave Freese, W1HKJ for creating fldigi
- Rik van Riel, AB1KW for the NAVTEX decoder


## Copyright

(C) 2021 Franco Venturi - Licensed under the GNU GPL V3 (see &lt;LICENSE&gt;)
