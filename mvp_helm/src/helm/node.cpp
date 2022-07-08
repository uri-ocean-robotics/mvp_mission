#include "helm.h"
#include "behavior_container.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "helm");

    helm::Helm obj;

    obj.initialize();

    obj.run();

    return 0;
}