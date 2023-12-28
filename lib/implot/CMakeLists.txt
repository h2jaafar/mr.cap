
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${mrcap_SOURCE_DIR}/include/implot/*.h")

set(IMPLOT_HEADER_LIST 
        ${mrcap_SOURCE_DIR}/include/implot/implot.h
        ${mrcap_SOURCE_DIR}/include/implot/implot_internal.h

        )

add_library(implot implot_items.cpp implot.cpp implot_demo.cpp ${IMPLOT_HEADER_LIST})

target_include_directories(implot PUBLIC ${mrcap_SOURCE_DIR}/include)
