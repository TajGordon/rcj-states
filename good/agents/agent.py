import config
from agents.blind_ball_chaser import BlindBallChaser

class Agent:
    def __init__(self, bot=None):
        """
        Initialize the agent with access to bot components.
        
        Args:
            bot: Bot instance with all components (camera, motors, etc.)
        """
        self.bot = bot
        self.blind_chaser = None
        
    def run(self, target_goal):
        """
        Run the agent strategy.
        
        Args:
            target_goal: 'blue' or 'yellow' - which goal to defend
        """
        if self.bot is None:
            print("ERROR: No bot instance provided to agent")
            return
            
        # For now, use the blind ball chaser
        self.blind_chaser = BlindBallChaser(self.bot)
        self.blind_chaser.run(target_goal)
        
    def stop(self):
        """Stop the agent."""
        if self.blind_chaser:
            self.blind_chaser.stop()