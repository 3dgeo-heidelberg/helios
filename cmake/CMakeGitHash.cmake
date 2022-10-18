# Prepare CURRENT_LIST_DIR to be a list of directories
set(CURRENT_LIST_DIR ${CMAKE_CURRENT_LIST_DIR})

# Assure ${pre_configure_dir} variable is defined
if(NOT DEFINED pre_configure_dir)
    set(pre_configure_dir src/main/)
endif()

# Assure ${post_configure_dir} variable is defined
if(NOT DEFINED post_configure_dir)
    set(post_configure_dir ../src/main/)
endif()

# Configure paths to input and output files containing the git hash
set(pre_configure_file ${pre_configure_dir}/helios_version.cpp.in)
set(post_configure_file ${post_configure_dir}/helios_version.cpp)

# Function to obtain the current git hash and update if different than latest
function(GitHashCompute)
    # Call git executable to obtain the git hash
    execute_process(
        COMMAND git log -1 --format=%h
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    # Read last git hash from file
    GitHashRead(GIT_HASH_CACHE)
    # Make dir if not exists
    if (NOT EXISTS ${post_configure_dir})
        file(MAKE_DIRECTORY ${post_configure_dir})
    endif()
    # Make file if not exists
    if (NOT EXISTS ${post_configure_dir}/helios_version.cpp)
        file(COPY ${pre_configure_dir}/helios_version.cpp
            DESTINATION ${post_configure_dir}
        )
    endif()
    # If there is no valid git hash
    if (NOT DEFINED GIT_HASH_CACHE)
        set(GIT_HASH_CACHE "INVALID")
    endif()
    # If git hash has changed, update helios_version.cpp. Otherwise, dont
    if (NOT ${GIT_HASH} STREQUAL ${GIT_HASH_CACHE} OR
        NOT EXISTS ${post_configure_file}
    )
        GitHashWrite(${GIT_HASH})
        # Replace any @Var@ in helios_version.cpp from helios_version.in
        configure_file(${pre_configure_file} ${post_configure_file} @ONLY)
    endif()
endfunction()

# Function to save the last git hash to a file
function(GitHashWrite GIT_HASH)
    file(WRITE ${CMAKE_BINARY_DIR}/last_GIT_HASH.txt ${GIT_HASH})
endfunction()

# Function to read the last git hash from a file
function(GitHashRead GIT_HASH)
    if (EXISTS ${CMAKE_BINARY_DIR}/last_GIT_HASH.txt)
        file(STRINGS ${CMAKE_BINARY_DIR}/last_GIT_HASH.txt CONTENT)
        LIST(GET CONTENT 0 last_GIT_HASH_from_file)
        set(${GIT_HASH} ${last_GIT_HASH_from_file} PARENT_SCOPE)
    endif()
endfunction()

# Function to call from CMakeLists
function(GitHashSetup)
    # Add a custom target to handle git hash on rebuild even without reconfigure
    add_custom_target(UpdateGitHash COMMAND ${CMAKE_COMMAND}
        -DRUN_CHECK_GIT_VERSION=1  # Run check git version function
        -Dpre_configure_dir=${pre_configure_dir}  # FROM config files
        -Dpost_configure_file=${post_configure_dir}  # TO config files
        -DGIT_HASH_CACHE=${GIT_HASH_CACHE}
        -P ${CURRENT_LIST_DIR}/CMakeGitHash.cmake  # Run without modify cache
        BYPRODUCTS ${post_configure_file}  # Prevent "file is missing" errors
    )
    add_library(git_hash src/main/helios_version.cpp)
    target_include_directories(git_hash PUBLIC src/main)
    add_dependencies(git_hash UpdateGitHash)
    GitHashCompute()
endfunction()