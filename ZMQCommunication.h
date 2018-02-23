#ifndef SOFA_CONTROLLER_ZMQCOMMUNICATION_H
#define SOFA_CONTROLLER_ZMQCOMMUNICATION_H

#include <SofaBaseTopology/TopologyData.h>
#include <SofaUserInteraction/Controller.h>

#include <zmq.hpp>
#include <string>

#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/vectorData.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <sstream>

#include "../initplugin.h"

namespace sofa
{
namespace core
{

namespace objectmodel
{

using core::objectmodel::Event;
using core::objectmodel::BaseObjectDescription;
using std::map;
using std::string;
using sofa::helper::vectorData;

template< class DataTypes >
class ZMQCommunication : public virtual sofa::core::objectmodel::BaseObject
{

  typedef sofa::defaulttype::Vec3d Vec3d;
  typedef sofa::defaulttype::Vec2d Vec2d;
  typedef sofa::defaulttype::Vec3f Vec3f;

 public:
  typedef BaseObject Inherited;
  SOFA_CLASS(SOFA_TEMPLATE(ZMQCommunication,DataTypes), Inherited);

  ZMQCommunication();
  virtual ~ZMQCommunication();

  sofa::Data<std::string> d_host;
  sofa::Data<ushort> d_port;

  sofa::Data<sofa::helper::vector<sofa::helper::vector<Vec2d>> > d_positions;
  sofa::Data<sofa::helper::vector<sofa::helper::vector<Vec2d>> > d_normals;
  sofa::Data<sofa::helper::vector<sofa::helper::vector<Vec2d>> > d_positionsmatched;
  sofa::Data<sofa::helper::vector<sofa::helper::vector<int>> > d_suppress;


  ////////////////////////// Inherited from BaseObject ////////////////////
  virtual void init() override;
  //virtual void reinit() override;
  //virtual void reset() override;
  /// Parse the given description to assign values to this object's fields and potentially other parameters
  //virtual void parse(BaseObjectDescription *arg) override;
  /// Assign the field values stored in the given map of name -> value pairs
  //virtual void parseFields(const map<string,string*>& str) override;
  void handleEvent(sofa::core::objectmodel::Event *event);
  /////////////////////////////////////////////////////////////////////////

  void serialize(std::stringstream& s);
  void desserialize(std::stringstream& s);
  void cleanup();

 private:
  zmq::context_t m_context{1};
  zmq::socket_t* m_sockSub;
  zmq::socket_t* m_sockPub;

};

}  // namespace controller
}  // namespace component
}  // namespace sofa

#endif  // SOFA_ZMQCOMMUNICATION_H
