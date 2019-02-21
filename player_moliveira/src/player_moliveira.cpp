#include <iostream>
#include <vector>
#include <ros/ros.h>

using namespace std;
using namespace boost;
using namespace ros;

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

        MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in) {
            team_red = (boost::shared_ptr<Team>) new Team("red");
            team_green = (boost::shared_ptr<Team>) new Team("green");
            team_blue = (boost::shared_ptr<Team>) new Team("blue");

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

            printInfo();
        }

        void printInfo(void)
        {
           ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name);
           ROS_INFO_STREAM("I am hunting " << team_preys->team_name << " and fleeing from " << team_hunters->team_name);
        }

    private:
    };



}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moliveira");
    ros::NodeHandle n;

    moliveira_ns::MyPlayer player("moliveira", "green");
    cout << "Hello World from " << player.player_name << " of team " << player.getTeamName() << endl;

    while(ros::ok())
    {
        ros::Duration(1).sleep();
        player.printInfo();
    }

    return 1;
}


