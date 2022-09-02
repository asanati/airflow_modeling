add_executable(airflow_data_collection src/main.cpp)

target_link_libraries(airflow_data_collection PUBLIC
  raptor_lib
  comm_lib
)