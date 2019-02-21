#include <iostream>
#include <vector>

using namespace std;

namespace moliveira_ns {

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
        MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in) {
            setTeamName(team_name_in);
        }

    private:
    };


    class Team
    {
        public:
        string team_name;
        vector<string> player_names;

        Team(string team_name_in)
        {
            team_name = team_name_in;
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

}

int main()
{
  moliveira_ns::MyPlayer player("moliveira", "green");
/*  player.setTeamName("blue");
  player.setTeamName(0);*/
  cout << "Hello World from " << player.player_name << " of team " << player.getTeamName() << endl;

  moliveira_ns::Team team_green("green");
  team_green.player_names.push_back("moliveira");
  team_green.player_names.push_back("blourenco");
  team_green.printInfo();
  cout << "tmadeira belongs to team? " << team_green.playerBelongsToTeam("tmadeira") << endl;
  return 1;
}


