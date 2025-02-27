/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/dataframe.pb.h>
#include <ignition/msgs/time.pb.h>

#include <memory>
#include <mutex>
#include <string>

#include <ignition/transport/Node.hh>
#include "ignition/gazebo/comms/Broker.hh"
#include "ignition/gazebo/comms/MsgManager.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Util.hh"

/// \brief Private Broker data class.
class ignition::gazebo::comms::Broker::Implementation
{
  /// \brief The message manager.
  public: MsgManager data;

  /// \brief Protect data from races.
  public: std::mutex mutex;

  /// \brief Topic used to centralize all messages sent from the agents.
  public: std::string msgTopic = "/broker/msgs";

  /// \brief Service used to bind to an address.
  public: std::string bindSrv = "/broker/bind";

  /// \brief Service used to unbind from an address.
  public: std::string unbindSrv = "/broker/unbind";

  /// \brief The current time.
  public: std::chrono::steady_clock::duration time{0};

  /// \brief An Ignition Transport node for communications.
  public: std::unique_ptr<ignition::transport::Node> node;
};

using namespace ignition;
using namespace gazebo;
using namespace comms;

//////////////////////////////////////////////////
Broker::Broker()
  : dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
  this->dataPtr->node = std::make_unique<ignition::transport::Node>();
}

//////////////////////////////////////////////////
void Broker::Load(std::shared_ptr<const sdf::Element> _sdf)
{
  if (!_sdf->HasElement("broker"))
    return;

  sdf::ElementPtr elem = _sdf->Clone()->GetElement("broker");
  this->dataPtr->msgTopic =
    elem->Get<std::string>("messages_topic", this->dataPtr->msgTopic).first;
  this->dataPtr->bindSrv =
    elem->Get<std::string>("bind_service", this->dataPtr->bindSrv).first;
  this->dataPtr->unbindSrv =
    elem->Get<std::string>("unbind_service", this->dataPtr->unbindSrv).first;
}

//////////////////////////////////////////////////
void Broker::Start()
{
  // Advertise the service for binding addresses.
  if (!this->dataPtr->node->Advertise(this->dataPtr->bindSrv,
                                      &Broker::OnBind, this))
  {
    ignerr << "Error advertising srv [" << this->dataPtr->bindSrv << "]"
           << std::endl;
    return;
  }

  // Advertise the service for unbinding addresses.
  if (!this->dataPtr->node->Advertise(this->dataPtr->unbindSrv,
                                      &Broker::OnUnbind, this))
  {
    ignerr << "Error advertising srv [" << this->dataPtr->unbindSrv << "]"
           << std::endl;
    return;
  }

  // Advertise the topic for receiving data messages.
  if (!this->dataPtr->node->Subscribe(this->dataPtr->msgTopic,
                                      &Broker::OnMsg, this))
  {
    ignerr << "Error subscribing to topic [" << this->dataPtr->msgTopic << "]"
           << std::endl;
    return;
  }

  igndbg << "Broker services:" << std::endl;
  igndbg << "  Bind: [" << this->dataPtr->bindSrv << "]" << std::endl;
  igndbg << "  Unbind: [" << this->dataPtr->unbindSrv << "]" << std::endl;
  igndbg << "Broker topics:" << std::endl;
  igndbg << "  Incoming messages: [" << this->dataPtr->msgTopic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
std::chrono::steady_clock::duration Broker::Time() const
{
  return this->dataPtr->time;
}

//////////////////////////////////////////////////
void Broker::SetTime(const std::chrono::steady_clock::duration &_time)
{
  this->dataPtr->time = _time;
}

//////////////////////////////////////////////////
bool Broker::OnBind(const ignition::msgs::StringMsg_V &_req,
                    ignition::msgs::Boolean &/*_rep*/)
{
  auto count = _req.data_size();
  if (count != 3)
  {
    ignerr << "Receive incorrect number of arguments. "
           << "Expecting 3 and receive " << count << std::endl;
    return false;
  }

  std::string address = _req.data(0);
  std::string model   = _req.data(1);
  std::string topic   = _req.data(2);

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->DataManager().AddSubscriber(address, model, topic))
    return false;

  ignmsg << "Address [" << address << "] bound to model [" << model
         << "] on topic [" << topic << "]" << std::endl;

  return true;
}

//////////////////////////////////////////////////
void Broker::OnUnbind(const ignition::msgs::StringMsg_V &_req)
{
  auto count = _req.data_size();
  if (count != 2)
  {
    ignerr << "Received incorrect number of arguments. "
           << "Expecting 2 and received " << count << std::endl;
    return;
  }

  std::string address = _req.data(0);
  std::string topic   = _req.data(1);

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->DataManager().RemoveSubscriber(address, topic);

  ignmsg << "Address [" << address << "] unbound on topic ["
         << topic << "]" << std::endl;
}

//////////////////////////////////////////////////
void Broker::OnMsg(const ignition::msgs::Dataframe &_msg)
{
  // Place the message in the outbound queue of the sender.
  auto msgPtr = std::make_shared<ignition::msgs::Dataframe>(_msg);

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Stamp the time.
  msgPtr->mutable_header()->mutable_stamp()->CopyFrom(
      gazebo::convert<msgs::Time>(this->dataPtr->time));

  this->DataManager().AddOutbound(_msg.src_address(), msgPtr);
}

//////////////////////////////////////////////////
void Broker::DeliverMsgs()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->DataManager().DeliverMsgs();
}

//////////////////////////////////////////////////
MsgManager &Broker::DataManager()
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
void Broker::Lock()
{
  this->dataPtr->mutex.lock();
}

//////////////////////////////////////////////////
void Broker::Unlock()
{
  this->dataPtr->mutex.unlock();
}
