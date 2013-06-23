/*  ModuleFactory
 *      Author: chris burbridge
 *
 * Creates instances of scitos components (factory pattern)
 */

#ifndef MODULEFACTORY_H_
#define MODULEFACTORY_H_

#include <map>
#include <string>
#include <ros/ros.h>

#include <scitos_driver/ScitosModule.h>
#include <scitos_driver/ScitosG5.h>

class ModuleFactory {
private:
	ModuleFactory();
    ModuleFactory(const ModuleFactory &) { }
    ModuleFactory &operator=(const ModuleFactory &) { return *this; }
    std::map<std::string, ModuleCreator> modules_;
public:
    ~ModuleFactory() { modules_.clear(); }
    static ModuleFactory *Get()
    {
        static ModuleFactory instance;
        return &instance;
    }

    void Register(const std::string &name, ModuleCreator create);
    ScitosModule *CreateModule(std::string name,  ScitosG5 *robot);
};

#endif /* MODULEFACTORY_H_ */

