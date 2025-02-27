/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_COMPONENTS_MODEL_HH_
#define IGNITION_GAZEBO_COMPONENTS_MODEL_HH_

#include <string>

#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace serializers
{
  class SdfModelSerializer
  {
    /// \brief Serialization for `sdf::Model`.
    /// \param[in] _out Output stream.
    /// \param[in] _time Model to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                const sdf::Model &_model)
    {
      sdf::ElementPtr modelElem = _model.Element();
      if (!modelElem)
      {
        ignwarn << "Unable to serialize sdf::Model" << std::endl;
        return _out;
      }

      bool skip = false;
      if (modelElem->HasElement("pose"))
      {
        sdf::ElementPtr poseElem = modelElem->GetElement("pose");
        if (poseElem->HasAttribute("relative_to"))
        {
          // Skip serializing models with //pose/@relative_to attribute
          // since deserialization will fail. This could be a nested model.
          // see https://github.com/ignitionrobotics/ign-gazebo/issues/1071
          // Once https://github.com/ignitionrobotics/sdformat/issues/820 is
          // resolved, there should be an API that returns sdf::Errors objects
          // instead of printing console msgs so it would be easier to ignore
          // specific errors in Deserialize.
          static bool warned = false;
          if (!warned)
          {
            ignwarn << "Skipping serialization / deserialization for models "
                    << "with //pose/@relative_to attribute."
                    << std::endl;
            warned = true;
          }
          skip = true;
        }
      }

      _out << "<?xml version=\"1.0\" ?>"
           << "<sdf version='" << SDF_PROTOCOL_VERSION << "'>"
           << (skip ? std::string() : modelElem->ToString(""))
           << "</sdf>";
      return _out;
    }

    /// \brief Deserialization for `sdf::Model`.
    /// \param[in] _in Input stream.
    /// \param[out] _model Model to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                sdf::Model &_model)
    {
      sdf::Root root;
      std::string sdf(std::istreambuf_iterator<char>(_in), {});

      sdf::Errors errors = root.LoadSdfString(sdf);
      if (!root.Model())
      {
        ignwarn << "Unable to deserialize sdf::Model" << std::endl;
        return _in;
      }

      _model = *root.Model();
      return _in;
    }
  };
}

namespace components
{
  /// \brief A component that identifies an entity as being a model.
  using Model = Component<NoData, class ModelTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.Model", Model)

  /// \brief A component that holds the model's SDF DOM
  using ModelSdf = Component<sdf::Model,
                   class ModelTag,
                   serializers::SdfModelSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.ModelSdf", ModelSdf)
}
}
}
}

#endif
