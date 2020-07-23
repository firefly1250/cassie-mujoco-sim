file(GLOB SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ./*.c*)

foreach(file ${SRCS})
    string(REPLACE ".c" "" name ${file})

    add_executable(${name} ${file})
    target_include_directories(${name} PUBLIC 
    )
    
    target_link_libraries(${name}
      cassiemujoco
    )
endforeach(file ${SRCS})
