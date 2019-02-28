#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <rws2019_msgs/MakeAPlay.h>
#include <rws2019_msgs/DoTheMath.h>

using namespace std;
using namespace boost;
using namespace ros;

float randomizePosition()
{
    srand(7786*time(NULL)); // set initial seed value to 5323
    return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

float randomizePosition2()
{
    srand(3521*time(NULL)); // set initial seed value to 5323
    return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

namespace moliveira_ns{

    class Team
    {
        public:
        string team_name;
        vector<string> player_names;
        ros::NodeHandle n;

        Team(string team_name_in)
        {
            team_name = team_name_in;
            //read the team players
            n.getParam("/team_" + team_name, player_names);
        }

        void printInfo()
        {
            cout << "Team " << team_name  << " has players: " << endl;
            for (size_t i=0; i<player_names.size(); cout << player_names[i++] << endl);
        }

        bool playerBelongsToTeam(string player_name)
        {
            for (size_t i=0; i<player_names.size(); i++)
            {
               if (player_names[i] == player_name)
                   return true;
            }

            return false;
        }

    private:
    };

    class Player {
    public:
        //properties
        string player_name;


        //methods
        Player(string player_name_in) {
            player_name = player_name_in;

        }

        void setTeamName(string team_name_in) {
            if (team_name_in == "red" || team_name_in == "green" || team_name_in == "blue") {
                team_name = team_name_in;
            } else {
                cout << "Cannot set team name " << team_name_in << endl;
            }
        }

        void setTeamName(int team_index) {
            if (team_index == 0) setTeamName("red");
            else if (team_index == 1) setTeamName("green");
            else if (team_index == 2) setTeamName("blue");
            else setTeamName("");
        }

        string getTeamName() { return team_name; };

    private:
        string team_name;

    };

    class MyPlayer : public Player {
    public:
        boost::shared_ptr<Team> team_red;
        boost::shared_ptr<Team> team_green;
        boost::shared_ptr<Team> team_blue;
        boost::shared_ptr<Team> team_hunters;
        boost::shared_ptr<Team> team_mine;
        boost::shared_ptr<Team> team_preys;
        tf::TransformBroadcaster br;
        tf::TransformListener listener;
        boost::shared_ptr<ros::Publisher> vis_pub;
        string last_prey;
        string last_hunter;

        MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in) {
            team_red = (boost::shared_ptr<Team>) new Team("red");
            team_green = (boost::shared_ptr<Team>) new Team("green");
            team_blue = (boost::shared_ptr<Team>) new Team("blue");

            ros::NodeHandle n;
            vis_pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
            (*vis_pub) = n.advertise<visualization_msgs::Marker>( "/bocas", 0 );

            if (team_red->playerBelongsToTeam(player_name))
            {
                team_mine = team_red;
                team_preys = team_green;
                team_hunters = team_blue;
            }else if (team_green->playerBelongsToTeam(player_name))
            {
                team_mine = team_green;
                team_preys = team_blue;
                team_hunters = team_red;
            }else if(team_blue->playerBelongsToTeam(player_name))
            {
                team_mine = team_blue;
                team_preys = team_red;
                team_hunters = team_green;
            }
            else{cout<<"something wrong in team parametrizations!!!" << endl;}

            setTeamName(team_mine->team_name);

            //define intial position
            float sx = randomizePosition();
            float sy = randomizePosition2();
            tf::Transform T1;
            T1.setOrigin( tf::Vector3(sx, sy, 0.0) );
            tf::Quaternion q;
            q.setRPY(0, 0, M_PI);
            T1.setRotation(q);

            //define global movement
            tf::Transform Tglobal = T1;
            br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));
            ros::Duration(0.1).sleep();
            br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));
            printInfo();

            last_prey = "";
            last_hunter = "";
        }

        void printInfo(void)
        {
           ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name);
           ROS_INFO_STREAM("I am hunting " << team_preys->team_name << " and fleeing from " << team_hunters->team_name);
        }

        std::tuple<float, float> getDistanceAndAngleToArena()
        {
            return getDistanceAndAngleToPlayer("world");
        }

        std::tuple<float, float> getDistanceAndAngleToPlayer(string other_player)
        {
            tf::StampedTransform T0;
            try{
                listener.lookupTransform(player_name, other_player, ros::Time(0), T0);
            }
            catch (tf::TransformException ex){
//                ROS_ERROR("%s",ex.what());
                ros::Duration(0.01).sleep();
                return {1000.0, 0.0};
            }

            float d = sqrt(T0.getOrigin().x() * T0.getOrigin().x() + T0.getOrigin().y() * T0.getOrigin().y() );
            float a = atan2( T0.getOrigin().y(), T0.getOrigin().x());
//            return std::tuple<float, float>(d,a);
            return {d,a};
        }

        void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
        {
//            bool something_changed = false;

            ROS_INFO("received a new msg");


            //STEP 1: Find out where I am
            tf::StampedTransform T0;
            try{
                listener.lookupTransform("/world", player_name, ros::Time(0), T0);
            }
            catch (tf::TransformException ex){
//                ROS_ERROR("%s",ex.what());
                ros::Duration(0.1).sleep();
            }

            //STEP 2: define how I want to move

            vector<float> distance_to_preys;
            vector<float> angle_to_preys;
            //For each prey find the closest. Then follow it
            for (size_t i =0; i< msg->red_alive.size(); i++)
            {
                std::tuple<float, float> t = getDistanceAndAngleToPlayer(msg->red_alive[i]);
                distance_to_preys.push_back( std::get<0>(t));
                angle_to_preys.push_back( std::get<1>(t));
            }

            //compute closest prey
            int idx_closest_prey = -1;
            float distance_closest_prey = 1000;
            for (size_t i =0; i< distance_to_preys.size(); i++)
            {
                if (distance_to_preys[i] < distance_closest_prey)
                {
                    idx_closest_prey = i;
                    distance_closest_prey = distance_to_preys[i];
                }
            }

            ROS_INFO_STREAM("idx_closest_prey = " << idx_closest_prey);

            float dx = 10;
            float a;

            if (idx_closest_prey != -1)
                a = angle_to_preys[idx_closest_prey];
            else
                a = M_PI;


            //Check if last_prey is different from prey
//            string prey = "";
//            if (idx_closest_prey != -1)
//            {
//               prey = msg->red_alive[idx_closest_prey];
//               if (prey != last_prey)
//               {
//                   something_changed = true;
//                   last_prey = prey;
//               }
//            }

            //STEP2.5: check values
            float dx_max = msg->dog;
            dx > dx_max ? dx = dx_max : dx = dx;

            double amax = M_PI/30;
            if (a!=0) {
                fabs(a) > fabs(amax) ? a = amax * a / fabs(a) : a = a;
            }

            //STEP 3: define local movement
            tf::Transform T1;
            T1.setOrigin( tf::Vector3(dx, 0.0, 0.0) );
            tf::Quaternion q;
            q.setRPY(0, 0, a);
            T1.setRotation(q);

            //STEP 4: define global movement
            tf::Transform Tglobal = T0*T1;
            br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));

//            if (something_changed) {
//                visualization_msgs::Marker marker;
//                marker.header.frame_id = player_name;
//                marker.header.stamp = ros::Time();
//                marker.ns = player_name;
//                marker.id = 0;
//                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//                marker.action = visualization_msgs::Marker::ADD;
//                marker.pose.position.y = 0.5;
//                marker.lifetime = ros::Duration(2);
//                marker.frame_locked = 1;
//                marker.scale.z = 0.4;
//                marker.color.a = 1.0; // Don't forget to set the alpha!
//                marker.color.r = 0.0;
//                marker.color.g = 0.0;
//                marker.color.b = 0.0;
//                marker.text = "tas tramado " + prey;
//
//                vis_pub->publish(marker);
//            }
        }


        bool doTheMathCallback(rws2019_msgs::DoTheMath::Request  &req,
                               rws2019_msgs::DoTheMath::Response &res)
        {
            if (req.op == "+")
                res.result = req.a + req.b;
            else if (req.op == "-")
                res.result = req.a - req.b;
            else if (req.op == "*")
                res.result = req.a * req.b;
            else if (req.op == "/")
                res.result = req.a / req.b;
            else
                return false;

            ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
            ROS_INFO("sending back response: [%ld]", (long int)res.result);
            return true;
        }


    private:
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moliveira");
    ros::NodeHandle n;

    moliveira_ns::MyPlayer player("moliveira", "green");
    ROS_INFO_STREAM("Initializing player " << "moliveira");

    ros::Subscriber sub = n.subscribe("/make_a_play", 100, &moliveira_ns::MyPlayer::makeAPlayCallback, &player);
    ros::ServiceServer service = n.advertiseService("do_the_math", &moliveira_ns::MyPlayer::doTheMathCallback, &player);
    ROS_INFO("Ready to do the math");

    player.printInfo();

    ros::Rate r(20);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 1;
}


