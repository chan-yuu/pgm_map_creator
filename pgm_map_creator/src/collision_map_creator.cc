/*
 * @Author: cyun
 * @Date: 2024-09-08 17:28:43
 * @LastEditors: fang
 * @LastEditTime: 2024-09-08 19:25:33
 * @FilePath: /undefined/home/kemove/catkin_ws6/src/pgm_map_creator/src/collision_map_creator.cc
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <fstream>
#include <iostream>
#include <math.h>
// #include <boost/gil/gil_all.hpp>
// #include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/gil.hpp>
#include <boost/gil/extension/io/png.hpp>

#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "collision_map_request.pb.h"
#include <iostream>
namespace gazebo
{
typedef const boost::shared_ptr<
  const collision_map_creator_msgs::msgs::CollisionMapRequest>
    CollisionMapRequestPtr;

class CollisionMapCreator : public WorldPlugin
{
  transport::NodePtr node;
  transport::PublisherPtr imagePub;
  transport::SubscriberPtr commandSubscriber;
  physics::WorldPtr world;
  // std::cout <<"start create"<< std::endl;

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    node = transport::NodePtr(new transport::Node());
    world = _parent;
    // Initialize the node with the world name
    node->Init(world->Name());
    std::cout << "Subscribing to: " << "~/collision_map/command" << std::endl;
    commandSubscriber = node->Subscribe("~/collision_map/command",
      &CollisionMapCreator::create, this);
    imagePub = node->Advertise<msgs::Image>("~/collision_map/image");
  }

  public: void create(CollisionMapRequestPtr &msg)
  {
    std::cout << "Received message" << std::endl;

    std::cout << "Creating collision map with corners at (" <<
      msg->upperleft().x() << ", " << msg->upperleft().y() << "), (" <<
      msg->upperright().x() << ", " << msg->upperright().y() << "), (" <<
      msg->lowerright().x() << ", " << msg->lowerright().y() << "), (" <<
      msg->lowerleft().x() << ", " << msg->lowerleft().y() <<
        ") with collision projected from z = " <<
      msg->height() << "\nResolution = " << msg->resolution() << " m\n" <<
        "Occupied spaces will be filled with: " << msg->threshold() <<
        std::endl;

    double dX_vertical = msg->upperleft().x() - msg->lowerleft().x();
    double dY_vertical = msg->upperleft().y() - msg->lowerleft().y();
    double mag_vertical =
      sqrt(dX_vertical * dX_vertical + dY_vertical * dY_vertical);
    dX_vertical = msg->resolution() * dX_vertical / mag_vertical;
    dY_vertical = msg->resolution() * dY_vertical / mag_vertical;

    double dX_horizontal = msg->upperright().x() - msg->upperleft().x();
    double dY_horizontal = msg->upperright().y() - msg->upperleft().y();
    double mag_horizontal =
      sqrt(dX_horizontal * dX_horizontal + dY_horizontal * dY_horizontal);
    dX_horizontal = msg->resolution() * dX_horizontal / mag_horizontal;
    dY_horizontal = msg->resolution() * dY_horizontal / mag_horizontal;

    int count_vertical = mag_vertical / msg->resolution();
    int count_horizontal = mag_horizontal / msg->resolution();

    if (count_vertical == 0 || count_horizontal == 0)
    {
      std::cout << "Image has a zero dimensions, check coordinates"
                << std::endl;
      return;
    }
    double x,y;

    boost::gil::gray8_pixel_t fill(255-msg->threshold());
    boost::gil::gray8_pixel_t blank(255);
    boost::gil::gray8_image_t image(count_horizontal, count_vertical);

    double dist;
    std::string entityName;
    ignition::math::Vector3d start, end;
    start.Z(msg->height());
    end.Z(0.001);

    gazebo::physics::PhysicsEnginePtr engine = world->Physics();
    engine->InitForThread();
    gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    std::cout << "Rasterizing model and checking collisions" << std::endl;
    boost::gil::fill_pixels(image._view, blank);

    for (int i = 0; i < count_vertical; ++i)
    {
      std::cout << "Percent complete: " << i * 100.0 / count_vertical
                << std::endl;
      x = i * dX_vertical + msg->lowerleft().x();
      y = i * dY_vertical + msg->lowerleft().y();
      for (int j = 0; j < count_horizontal; ++j)
      {
        x += dX_horizontal;
        y += dY_horizontal;

        start.X(x);
        end.X(x);
        start.Y(y);
        end.Y(y);
        ray->SetPoints(start, end);
        ray->GetIntersection(dist, entityName);
        if (!entityName.empty())
        {
          image._view(i,j) = fill;
        }
      }
    }

    std::cout << "Completed calculations, writing to image" << std::endl;
    if (!msg->filename().empty())
    {
        boost::gil::gray8_view_t view = image._view;

        // Assuming Gazebo origin is at (0, 0, 0) in world coordinates
        ignition::math::Vector3d gazebo_origin(0.0, 0.0, 0.0);  // x, y, theta

        // Write to pgm and yaml
        pgm_write_view(msg->filename(), view, msg->filename() + ".yaml", msg->resolution(), gazebo_origin);
    }


    std::cout << "Output location: " << msg->filename() << std::endl;
  }

  // public: void pgm_write_view(const std::string& filename, boost::gil::gray8_view_t& view)
  // {
  //   // Write image to pgm file
  //   std::cout << "running" << std::endl;
  //   int h = view.height();
  //   int w = view.width();

  //   std::ofstream ofs;
  //   ofs.open(filename+".pgm");
  //   ofs << "P2" << '\n';          // grayscale
  //   ofs << w << ' ' << h << '\n'; // width and height
  //   ofs << 255 <<  '\n';          // max value
  //   for (int y = 0; y < h; ++y){
  //     for (int x = 0; x < w; ++x){
  //       // std::cout << (int)view(x, y)[0];
  //       ofs << (int)view(x, y)[0] << ' ';
  //     }
  //     ofs << '\n';
  //   }
  //   ofs.close();
  // }
  public: void pgm_write_view(const std::string& filename, boost::gil::gray8_view_t& view, const std::string& yaml_filename, double resolution, const ignition::math::Vector3d& gazebo_origin)
  {
      // Write image to pgm file
      std::cout << "Writing image to " << filename << ".pgm" << std::endl;
      int h = view.height();
      int w = view.width();

      std::ofstream ofs;
      ofs.open(filename + ".pgm");
      ofs << "P2" << '\n';          // Grayscale
      ofs << w << ' ' << h << '\n'; // Width and height
      ofs << 255 <<  '\n';          // Max value
      for (int y = 0; y < h; ++y) {
          for (int x = 0; x < w; ++x) {
              ofs << (int)view(x, y)[0] << ' ';
          }
          ofs << '\n';
      }
      ofs.close();

      // Calculate the origin in the Gazebo world
      // double origin_x = gazebo_origin.X() - (w / 2.0) * resolution;
      // double origin_y = gazebo_origin.Y() - (h / 2.0) * resolution;
      // double origin_theta = gazebo_origin.Z();  // If you want rotation
      double origin_x = ((w / 2.0) * resolution);
      double origin_y = -((h / 2.0) * resolution);
      double origin_theta = M_PI / 2;  // Assuming no rotation, can be modified if needed
      // std::cout<<"origin_x"<<origin_x<<"origin_y"<<origin_y<<std::endl;
      // Write YAML file
      std::cout << "Writing metadata to " << yaml_filename << std::endl;
      std::ofstream yaml_ofs;
      yaml_ofs.open(yaml_filename);
      yaml_ofs << "image: " << filename << ".pgm" << '\n';
      yaml_ofs << "resolution: " << resolution << '\n';
      yaml_ofs << "origin: [" << origin_x << ", " << origin_y << ", " << origin_theta << "]" << '\n';
      yaml_ofs << "negate: 0" << '\n';
      yaml_ofs << "occupied_thresh: 0.65" << '\n';
      yaml_ofs << "free_thresh: 0.196" << '\n';
      yaml_ofs.close();
  }

};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CollisionMapCreator)
}
