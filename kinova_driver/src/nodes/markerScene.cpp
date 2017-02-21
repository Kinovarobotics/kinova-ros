/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int f = 1;
bool fup = true;

visualization_msgs::Marker updateMarkers(float offset)
{
// %Tag(MARKER_INIT)%
    visualization_msgs::Marker points; //, line_strip, line_list;
    points.header.frame_id = "/map";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w =  1.0;
// %EndTag(MARKER_INIT)%

// %Tag(ID)%
    points.id = 0;
// %EndTag(ID)%

// %Tag(TYPE)%
    points.type = visualization_msgs::Marker::POINTS;

// %EndTag(TYPE)%

// %Tag(SCALE)%
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
// %Tag(COLOR)%
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

// %Tag(HELIX)%
    // Create the vertices for the points and lines
      float y = 0.38 + offset;
      float z = 0.22;
      float x = 0.33;

      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
     // line_strip.points.push_back(p);

      // The line list needs two points for each line

// %EndTag(HELIX)%

   return points;
}

visualization_msgs::Marker run()
{ if (fup == true && f <=5)
   { f += 1;}
  else if (fup == false && f >= -5)
    { f -= 1;}
  else if (f>4)
    { fup = false; }
  else if (f<-4)
    { fup = true; }  
  else {f = 0;}

 visualization_msgs::Marker a = updateMarkers(f);
 return a;
}
int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher Targ_pub = n.advertise<visualization_msgs::Marker>("Targ_marker", 10);

  ros::Rate r(30);

  //  visualization_msgs::Marker pointer;

  while (ros::ok())
  {

    marker_pub.publish(updateMarkers(0));
    Targ_pub.publish(run());

    r.sleep();

  }
}
// %EndTag(FULLTEXT)%


