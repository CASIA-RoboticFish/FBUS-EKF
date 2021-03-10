#ifndef YAML_EIGEN_CONVERTER_HPP
#define YAML_EIGEN_CONVERTER_HPP

#include <iostream>
#include <fstream> 
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace YAML{

    // template<>
    // struct convert<std::vector<int>>{
    //     static bool decode(const Node& node, std::vector<int>& IntList){
    //         if(!node.IsSequence()){
    //             return false;
    //         }
    //         for(int i=0; i < 3; i++){
    //             for(int j=0; j < 3; j++){
    //                 Matrix(i,j) = node[i][j].as<double>();
    //             }
    //         }
    //     }
    // };

    template<>
    struct convert<Eigen::Matrix3d>{
        static bool decode(const Node& node, Eigen::Matrix3d& Matrix){
            if(!node.IsSequence()){
                return false;
            }
            for(int i=0; i < 3; i++){
                for(int j=0; j < 3; j++){
                    Matrix(i,j) = node[i][j].as<double>();
                }
            }
        }
    };

    template<>
    struct convert<Eigen::Matrix4d>{
        static bool decode(const Node& node, Eigen::Matrix4d& Matrix){
            if(!node.IsSequence()){
                return false;
            }
            for(int i=0; i < 4; i++){
                for(int j=0; j < 4; j++){
                    Matrix(i,j) = node[i][j].as<double>();
                }
            }
        }
    };

    template<>
    struct convert<Eigen::Matrix<double,3,4>>{
        static bool decode(const Node& node, Eigen::Matrix<double,3,4>& Matrix){
            if(!node.IsSequence()){
                return false;
            }
            for(int i=0; i < 3; i++){
                for(int j=0; j < 4; j++){
                    Matrix(i,j) = node[i][j].as<double>();
                }
            }
        }
    };

    // template<>
    // struct convert<Eigen::Quaterniond>{
    //     static bool decode(const Node& node, Eigen::Quaterniond& quat){
    //         if(!node.IsSequence()){
    //             return false;
    //         }
    //         Eigen::Quaterniond q(node[0][0].as<double>(), node[1][0].as<double>(), node[2][0].as<double>(), node[3][0].as<double>());
    //         quat = q;
    //     }
    // };

    template<>
    struct convert<Eigen::Vector4d>{
        static bool decode(const Node& node, Eigen::Vector4d& Vec){
            if(!node.IsSequence()){
                return false;
            }
            for(int i=0; i < 4; i++){
                    Vec(i) = node[i][0].as<double>();
            }
        }
    };

    template<>
    struct convert<Eigen::Vector3d>{
        static bool decode(const Node& node, Eigen::Vector3d& Vec){
            if(!node.IsSequence()){
                return false;
            }
            for(int i=0; i < 3; i++){
                    Vec(i) = node[i][0].as<double>();
            }
        }
    };

    template<>
    struct convert<Eigen::Vector2d>{
        static bool decode(const Node& node, Eigen::Vector2d& Vec){
            if(!node.IsSequence()){
                return false;
            }
            for(int i=0; i < 2; i++){
                    Vec(i) = node[i][0].as<double>();
            }
        }
    };
};

#endif