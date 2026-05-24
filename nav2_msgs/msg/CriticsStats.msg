# nav2_msgs/msg/CriticsStats.msg
std_msgs/Header header
string[]  critics_names       # name of each critic plugin
bool[]    critics_triggered   # did it add any cost this iteration? (delta > 1e-6)
float64[] costs_sum           # raw cost delta added by this critic across all trajectories
float64[] cost_percentage     # this critic's % share of total_cost this cycle (0-100)
float64[] cost_normalized     # costs_sum / weight: situation severity, independent of tuning
float64[] trigger_rate        # rolling fraction of cycles where this critic fired (0.0-1.0)
float64   total_cost          # sum of all critics' costs_sum this cycle
string    dominant_critic     # name of the critic with the highest cost_percentage