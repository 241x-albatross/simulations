from simulator import flight_sim
from rllab.envs.base import Env
from rllab.spaces import Box
from rllab.envs.base import Step
import numpy as np


def wrapAngle(x):
    while x < -np.pi:
        x += 2*np.pi
    while x > np.pi:
        x -= 2*np.pi
    return x


dt_ = 0.5
WAYPOINT_REWARD = 1
TURN_COST = 0.1
ALTITUDE_COST = 10
VELOCITY_COST = 10
class FlightEnv(object):
    def __init__(self):
        self.mission = [[200,100], [0,200], [100,15], [100,300]]
        self.radius = 50

    def observation(self):
        """
        Current altitude, pitch, roll
        relative distance and course to next two waypoints
        relative course error between desired direction
        """
        nextwp = self.mission[0]
        dist = np.sqrt( (self.x_[0] - nextwp[0])**2 + (self.x_[1] - nextwp[1])**2 )
        course = wrapAngle(np.arctan2(self.x_[1] - nextwp[1], self.x_[0] - nextwp[0]))
        course_error = wrapAngle(self.x_[0] - course)
        if len(self.mission) <= 1:
            obs = [self.x_[3], self.x_[4], self.x_[6], dist, course, course_error, 1000, course, course_error]
        else:
            nextwp2 = self.mission[1]
            dist2 = np.sqrt( (self.x_[0] - nextwp2[0])**2 + (self.x_[1] - nextwp2[1])**2 )
            course2 = wrapAngle(np.arctan2(self.x_[1] - nextwp2[1], self.x_[0] - nextwp2[0]))
            course_error2 = wrapAngle(self.x_[0] - course2)
            obs = [self.x_[3], self.x_[4], self.x_[6], dist, course, course_error, dist2, course2, course_error2]

        return obs

    def step(self, action):
        """
        Run one timestep of the environment's dynamics. When end of episode
        is reached, reset() should be called to reset the environment's internal state.
        Input
        -----
        action : an action provided by the environment
        Outputs
        -------
        (observation, reward, done, info)
        observation : agent's observation of the current environment
        reward [Float] : amount of reward due to the previous action
        done : a boolean, indicating whether the episode has ended
        info : a dictionary containing other diagnostic information from the previous action
        """
        u = action
        w = np.zeros(3)
        dx = flight_sim( self.x_, u, w )
        self.x_ = self.x_ + dx*dt_
        obs = self.observation()

        # compute reward
        reward = 0
        if obs[3] < self.radius:
            reward += WAYPOINT_REWARD
            self.mission = self.mission[1:]
        if abs(u[1]) + abs(u[2]) > 0.3:
            reward -= TURN_COST
        if self.x_[2] < 20:
            reward -= ALTITUDE_COST
        if self.x_[5] < 5 or self.x_[5] > 20:
            reward -= VELOCITY_COST

        done = False
        if len(self.mission) == 0:
            done = True
        return Step(observation=obs, reward=reward, done=done)

    def reset(self):
        """
        Resets the state of the environment, returning an initial observation.
        Outputs
        -------
        observation : the initial observation of the space. (Initial reward is assumed to be 0.)
        """
        self.mission = [[200,100], [0,200], [100,15], [100,300]]
        x0 = 500*np.random.rand()
        y0 = 500*np.random.rand()
        h0 = 20+50*np.random.rand()
        chi0 = -np.pi + 2*np.pi*np.random.rand()
        gamma0 = 0
        Va0 = 15
        phi0 = 0
        self.x_ = [x0,y0,h0,chi0,gamma0,Va0,phi0]
        return self.observation()

    def render(self):
        print('current state', self.x_)

    @property
    def action_space(self):
        """
        Returns a Space object
        """
        low = np.array([5, -np.pi/8, -np.pi/4])
        high = np.array([20, np.pi/8, np.pi/4])
        return Box(low=low, high=high)

    @property
    def observation_space(self):
        """
        Returns a Space object
        """
        low = np.array([0, -np.pi/2, -np.pi/2, 0, -np.pi, -np.pi, 0, -np.pi, -np.pi])
        high = np.array([100, np.pi/2, np.pi/2, 1000, np.pi, np.pi, 1000, np.pi, -np.pi])
        return Box(low=low, high=high)

    def log_diagnostics(self, paths):
        pass


if __name__ == "__main__":
    from rllab.algos.trpo import TRPO
    from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
    from rllab.envs.normalized_env import normalize
    from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy

    env = normalize(FlightEnv())
    policy = GaussianMLPPolicy(
        env_spec=env.spec,
    )
    baseline = LinearFeatureBaseline(env_spec=env.spec)
    algo = TRPO(
        env=env,
        policy=policy,
        baseline=baseline,
        max_path_length=400,
        batch_size=4000,
        gae_lambda=0.7
    )
    algo.train()

    
