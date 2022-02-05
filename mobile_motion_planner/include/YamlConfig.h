#pragma once
#include <yaml-cpp/yaml.h>

struct YAMLConfig
{
  void loadConfig(std::string file_name)
  {
    YAML::Node yamlnode;
    //cout << file_name << endl;


    yamlnode = YAML::LoadFile(file_name);

    // urdf_path = yamlnode["urdf_path"].as<std::string>();

    // chain_start = yamlnode["chain_start"].as<std::string>();
    // chain_end = yamlnode["chain_end"].as<std::string>();

    position_limit_lower = yamlnode["position_limit_lower"].as<std::vector<double> >();
    position_limit_upper = yamlnode["position_limit_upper"].as<std::vector<double> >();

    mobile_length = yamlnode["mobile_length"].as<double >();
    mobile_width = yamlnode["mobile_width"].as<double >();

    //link_name = yamlnode["link_data"]["link_name"].as<std::vector<string>>();
    //link_dimension = yamlnode["link_data"]["link_dimension"].as<std::vector<std::vector<double>> >();
  }

  std::string urdf_path;
  std::string chain_start;
  std::string chain_end;

  std::vector<double> position_limit_lower;
  std::vector<double> position_limit_upper;

  double mobile_length;
  double mobile_width;

  std::vector<std::string> link_name;
  std::vector<std::vector<double>> link_dimension;


};