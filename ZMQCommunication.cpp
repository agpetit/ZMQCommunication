#ifndef SOFA_CONTROLLER_ZMQCOMMUNICATION_CPP
#define SOFA_CONTROLLER_ZMQCOMMUNICATION_CPP

#include "ZMQCommunication.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>


namespace sofa
{
namespace core
{
namespace objectmodel
{

using sofa::defaulttype::Vec3d;
using sofa::defaulttype::Vec2d;
using sofa::defaulttype::Vec3f;
using sofa::defaulttype::Vec2f;
using sofa::defaulttype::Vec1d;
using sofa::defaulttype::Vec1f;
using sofa::defaulttype::Vec;
using helper::vector;

using sofa::defaulttype::Rigid3dTypes;
using sofa::defaulttype::Rigid3fTypes;


template<class DataTypes>
ZMQCommunication<DataTypes>::ZMQCommunication()
    : Inherited()
    , d_host(initData(&d_host, std::string("127.0.0.1"), "host",
                      "hostname to connect to")),
      d_port(initData(&d_port, ushort(8888), "port", "port to connect to")),
      d_positions(initData(&d_positions, "positions", "2D Positions to send over the network")),
      d_normals(initData(&d_normals, "normals", "2D normals to send over the network")),
      d_positionsmatched(initData(&d_positionsmatched, "matchedpositions", "2D matched points retrieved from the network")),
      d_suppress(initData(&d_suppress, "suppress", "Valid matched points = 0, otherwise = 1 "))
{

}

template<class DataTypes>
ZMQCommunication<DataTypes>::~ZMQCommunication()
{
cleanup();
}


template<class DataTypes>
void ZMQCommunication<DataTypes>::init()
{

  std::stringstream serialized;
  serialize(serialized);
  m_sockPub = new zmq::socket_t(m_context, ZMQ_PUB);
  m_sockPub->bind("tcp://127.0.0.1:6667");

  m_sockSub = new zmq::socket_t(m_context, ZMQ_SUB);
  m_sockSub->setsockopt(ZMQ_SUBSCRIBE, "", 0);
  m_sockSub->connect("tcp://127.0.0.1:6666");
}

template<class DataTypes>
void ZMQCommunication<DataTypes>::serialize(std::stringstream& s)
{

  double t = (double)this->getContext()->getTime();
  // 2d positions

sofa::helper::vector<sofa::helper::vector<Vec2d>> d_poss;
sofa::helper::vector<sofa::helper::vector<Vec2d>> d_norms;


sofa::helper::vector<Vec2d> d_pos;
sofa::helper::vector<Vec2d> d_norm;
Vec2d pos,norm;
pos[0] = 0.1;
pos[1] = 0.5;
norm[0] = 0.2;
norm[1] = 0.3;
d_pos.push_back(pos);
d_norm.push_back(norm);
d_poss.push_back(d_pos);
d_norms.push_back(d_norm);
d_positions.setValue(d_poss);
d_normals.setValue(d_norms);

for (int k=0; k < d_positions.getValue().size(); k++)
{
  s << d_positions.getValue()[k];
  s << ";";

  s << d_normals.getValue()[k];
  s << ";";
}
}

template<class DataTypes>
void ZMQCommunication<DataTypes>::desserialize(std::stringstream& s)
{


    sofa::helper::vector<sofa::helper::vector<Vec2d>> positionsmatched;
    sofa::helper::vector<sofa::helper::vector<int>> suppress;

    positionsmatched.resize(0);
    suppress.resize(0);

    int nimages = 0;

    while (!s.eof())
    {
           sofa::helper::vector<Vec2d> positionsmatched0;
           positionsmatched0.resize(0);

           sofa::helper::vector<int> suppress0;
           suppress0.resize(0);

           int npoints = 0;

             while (s.peek()!=';')
             {
                 Vec2d point;
                 int x,y;
                 s >> x;
                 s >> y;
                 point[0] = (double)x;
                 point[1] = (double)y;
                 npoints ++;
                 //std::cout << " pos matched " << x << " " << y << std::endl;
                 positionsmatched0.push_back(point);
                 s.ignore();
             }

             s.ignore();
             positionsmatched.push_back(positionsmatched0);
             nimages++;

             while (s.peek()!=';')
             {

                 int supp;
                 s >> supp;
                 //std::cout << " supp " << supp << std::endl;
                 suppress0.push_back(supp);
                 s.ignore();
             }
             s.ignore();
             s.ignore();
             suppress.push_back(suppress0);
             //std::cout << "stream " << std::endl;
    }


d_positionsmatched.setValue(positionsmatched);
d_suppress.setValue(suppress);


}

template<class DataTypes>
void ZMQCommunication<DataTypes>::handleEvent(sofa::core::objectmodel::Event *event)
{
  std::stringstream serialized;
  this->serialize(serialized);

  zmq::message_t message(serialized.str().length());
  std::memcpy(message.data(), serialized.str().c_str(),
              serialized.str().length());

  bool status = m_sockPub->send(message);

  if (!status) msg_error(getName() + "::update()") << "could not send message";

  zmq::message_t message1;

  status = m_sockSub->recv(&message1);

  char messageChar1[message1.size()];
  memcpy(&messageChar1, message1.data(), message1.size());

  std::stringstream stream1;
  unsigned int nbDataFieldReceived = 0;

  for(unsigned int i=0; i<message1.size(); i++)
      stream1 << messageChar1[i];


  if (status)
  {
    desserialize(stream1);
  }
  else
    msg_error(getName() + "::update()") << "could not retrieve message";
}

template<class DataTypes>
void ZMQCommunication<DataTypes>::cleanup()
{
  if (m_sockSub)
  {
      m_sockSub->close();
      delete m_sockSub;
      m_sockSub = NULL;
  }
  //if (m_sockPub->connected())
  {
      m_sockPub->close();
      delete m_sockPub;
      m_sockPub = NULL;
  }
}

////////////////////////////////////////////    FACTORY    ////////////////////////////////////////////
using sofa::core::RegisterObject ;

// Registering the component
SOFA_DECL_CLASS(ZMQCommunication)

int ZMQCommunicationClass = RegisterObject("This component is used to build a communication between two simulations")

#ifdef SOFA_WITH_DOUBLE
.add< ZMQCommunication<double> >(true)
.add< ZMQCommunication<vector<Vec3d>> >()
.add< ZMQCommunication<vector<Vec1d>> >()
.add< ZMQCommunication<vector<Rigid3dTypes::Coord>> >()
.add< ZMQCommunication<vector<vector<Vec2d>>> >()
#endif
#ifdef SOFA_WITH_FLOAT
.add< ZMQCommunication<float> >()
.add< ZMQCommunication<vector<Vec3f>> >()
.add< ZMQCommunication<vector<Vec1f>> >()
.add< ZMQCommunication<vector<Rigid3fTypes::Coord>> >()
.add< ZMQCommunication<vector<vector<Vec2f>>> >()

#endif
.add< ZMQCommunication<int> >()
.add< ZMQCommunication<unsigned int> >()
.add< ZMQCommunication<vector<Vec<2,int>>> >()
.add< ZMQCommunication<vector<Vec<2,unsigned int>>> >()
.add< ZMQCommunication<vector<vector<Vec<2,unsigned int>>>> >()
;

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Force template specialization for the most common sofa floating point related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_WITH_DOUBLE
template class ZMQCommunication<double>;
template class ZMQCommunication<vector<Vec3d>>;
template class ZMQCommunication<vector<Vec1d>>;
template class ZMQCommunication<vector<Rigid3dTypes::Coord>>;
template class  ZMQCommunication<vector<vector<Vec2d>>>;
#endif
#ifdef SOFA_WITH_FLOAT
template class ZMQCommunication<float>;
template class ZMQCommunication<vector<Vec3f>>;
template class ZMQCommunication<vector<Vec1f>>;
template class ZMQCommunication<vector<Rigid3fTypes::Coord>>;
template class ZMQCommunication<vector<vector<Vec2f>>>;
#endif
template class ZMQCommunication<int>;
template class ZMQCommunication<unsigned int>;
template class ZMQCommunication<vector<Vec<2,int>> >;
template class ZMQCommunication<vector<vector<Vec<2,int>>>>;

}  // namespace fusion
}  // namespace sofa
}

#endif
