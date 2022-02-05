#pragma once
#include <yaml-cpp/yaml.h>

struct YAMLConfig
{
  void loadConfig(std::string file_name)
  {
    YAML::Node yamlnode;
    yamlnode = YAML::LoadFile(file_name);

    urdf_path = yamlnode["urdf_path"].as<std::string>();

    chain_start = yamlnode["chain_start"].as<std::string>();
    chain_end = yamlnode["chain_end"].as<std::string>();

    joint_limit_lower = yamlnode["joint_limit_lower"].as<std::vector<double> >();
    joint_limit_upper = yamlnode["joint_limit_upper"].as<std::vector<double> >();

    link_name = yamlnode["link_data"]["link_name"].as<std::vector<std::string>>();
    link_dimension = yamlnode["link_data"]["link_dimension"].as<std::vector<std::vector<double>> >();
    link_position = yamlnode["link_data"]["link_position"].as<std::vector<std::vector<double>> >();
    link_orientation = yamlnode["link_data"]["link_orientation"].as<std::vector<std::vector<double>> >();
  }

  std::string urdf_path;

  std::string chain_start;
  std::string chain_end;

  std::vector<double> joint_limit_lower;
  std::vector<double> joint_limit_upper;

  std::vector<std::string> link_name;
  std::vector<std::vector<double>> link_dimension;
  std::vector<std::vector<double>> link_position;
  std::vector<std::vector<double>> link_orientation;


};