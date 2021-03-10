#include <iostream>
#include <yaml-cpp/yaml.h>


// void operator >> (const YAML::Node& node, Monster& monster) {
//    node["name"] >> monster.name;
//    node["position"] >> monster.position;
//    const YAML::Node& powers = node["powers"];
//    for(unsigned i=0;i<powers.size();i++) {
//       Power power;
//       powers[i] >> power;
//       monster.powers.push_back(power);
//    }
// }

int main(int agec, char **argv)
{
    YAML::Node markerGroupFileNode = YAML::LoadFile("config/markersetup.yml");
    YAML::Node markerGroupNode = markerGroupFileNode["Marker Group"];
    // const YAML::Node markers = markerGroupNode["Marker"];
    std::cout << markerGroupNode.size() << std::endl;
    for(unsigned i=0;i<markerGroupNode.size();i++) {
        YAML::Node marker = markerGroupNode[i];
        double id = marker["id"].as<double>();
        std::cout << "Marker "<< id << std::endl;

    }
    return 0;
}