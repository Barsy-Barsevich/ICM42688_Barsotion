# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS main.c ../src/ICM42688_Barsotion.c ../src/ICM42688_SPI.c ../src/ICM42688_I2C.c ../src/ICM42688_Filter.c ../src/microFFT.c        # list the source files of this component
    INCLUDE_DIRS        #../src # optional, add here public include directories
    PRIV_INCLUDE_DIRS   # optional, add here private include directories
    REQUIRES            # optional, list the public requirements (component names)
    PRIV_REQUIRES       # optional, list the private requirements
    #SRC_DIRS			../src
    #EMBED_TXTFILES 		../src/CMakeLists.txt
)
