# Add the subdirectory that contains the library
add_subdirectory(lib)

# Create the executable using main.c
add_executable(my_app main.c)

# Link the executable with the static library "mylib"
target_link_libraries(my_app PRIVATE mylib)
