set(PROJECT arch)

set(APP_SOURCES
    src/DataAccessor.cpp
    src/Fibre.cpp
	)

add_library (${PROJECT} STATIC ${APP_SOURCES})
target_include_directories (${PROJECT} PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR}/inc
	)
	
IF(TEST)
	set(TEST_SOURCES
	    unit/Fibre_t.cpp
		)	
	add_executable (arch_test ${TEST_SOURCES} ${APP_SOURCES})
	target_link_libraries(arch_test gtest_main ${PROJECT})
	add_test(arch_test arch_test) 
ENDIF()

