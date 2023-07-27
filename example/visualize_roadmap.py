#!/usr/bin/env python3
import yaml
import matplotlib
# matplotlib.use("Agg")
from matplotlib.patches import Circle, FancyArrow, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
import argparse
import math

Colors = ['orange', 'blue', 'green']


class Animation:
  def __init__(self, map, schedule, radius):
    self.map = map
    self.schedule = schedule
    self.radius = radius

    # find boundary
    all_pos = np.array(list(map["roadmap"]["vertices"].values()))
    print(all_pos)
    xmin = np.min(all_pos[:, 0])
    ymin = np.min(all_pos[:, 1])
    xmax = np.max(all_pos[:, 0])
    ymax = np.max(all_pos[:, 1])

    aspect = (xmax - xmin) / (ymax - ymin)

    self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
    self.ax = self.fig.add_subplot(111, aspect='equal')
    self.fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)

    self.patches = []
    self.artists = []
    self.agents = dict()
    self.agent_names = dict()

    plt.xlim(xmin - radius, xmax + radius)
    plt.ylim(ymin - radius, ymax + radius)

    v_dict = map["roadmap"]["vertices"]
    for edge in map["roadmap"]["edges"]:
      start = v_dict[edge[0]]
      goal = v_dict[edge[1]]
      self.patches.append(FancyArrow(start[0], start[1], goal[0] - start[0], goal[1] - start[1], width=0.05, length_includes_head=True))#, head_width=0))

    # create agents:
    self.T = 0
    # draw goals first
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      if "goal" in d:
        goals = [d["goal"]]
      if "potentialGoals" in d:
        goals = [goal for goal in d["potentialGoals"]]
      for goal in goals:
        v = v_dict[goal]
        self.patches.append(Rectangle((v[0] - radius, v[1] - radius), 2*radius, 2*radius, facecolor=Colors[i%len(Colors)], edgecolor='black', alpha=0.5))

    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
      name = d["name"]
      v = v_dict[d["start"]]
      self.agents[name] = Circle((v[0], v[1]), radius, facecolor=Colors[i%len(Colors)], edgecolor='black')
      self.agents[name].original_face_color = Colors[i%len(Colors)]
      self.patches.append(self.agents[name])
      self.T = max(self.T, schedule["schedule"][name][-1]["t"])
      self.agent_names[name] = self.ax.text(v[0], v[1], name.replace('agent', ''))
      self.agent_names[name].set_horizontalalignment('center')
      self.agent_names[name].set_verticalalignment('center')
      self.artists.append(self.agent_names[name])

    self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                               init_func=self.init_func,
                               frames=int(self.T+1) * 10,
                               interval=100,
                               blit=True)

  def save(self, file_name, speed):
    self.anim.save(
      file_name,
      "ffmpeg",
      fps=10 * speed,
      dpi=200),
      # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

  def show(self):
    plt.show()

  def init_func(self):
    for p in self.patches:
      self.ax.add_patch(p)
    for a in self.artists:
      self.ax.add_artist(a)
    return self.patches + self.artists

  def animate_func(self, i):
    for agent_name in self.schedule["schedule"]:
      agent = self.schedule["schedule"][agent_name]
      pos = self.getState(i / 10, agent)
      p = (pos[0], pos[1])
      self.agents[agent_name].center = p
      self.agent_names[agent_name].set_position(p)

    # reset all colors
    for _,agent in self.agents.items():
      agent.set_facecolor(agent.original_face_color)

    # check drive-drive collisions
    agents_array = [agent for _,agent in self.agents.items()]
    for i in range(0, len(agents_array)):
      for j in range(i+1, len(agents_array)):
        d1 = agents_array[i]
        d2 = agents_array[j]
        pos1 = np.array(d1.center)
        pos2 = np.array(d2.center)
        if np.linalg.norm(pos1 - pos2) < 2 * self.radius:
          d1.set_facecolor('red')
          d2.set_facecolor('red')
          print("COLLISION! (agent-agent) ({}, {})".format(i, j))

    return self.patches + self.artists


  def getState(self, t, d):
    v_dict = self.map["roadmap"]["vertices"]
    idx = 0
    while idx < len(d) and d[idx]["t"] < t:
      idx += 1
    if idx == 0:
      return np.array(v_dict[d[0]["v"]])
    elif idx < len(d):
      posLast = np.array(v_dict[d[idx-1]["v"]])
      posNext = np.array(v_dict[d[idx]["v"]])
    else:
      return np.array(v_dict[d[-1]["v"]])
    dt = d[idx]["t"] - d[idx-1]["t"]
    t = (t - d[idx-1]["t"]) / dt
    pos = (posNext - posLast) * t + posLast
    return pos

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("schedule", help="schedule for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
  parser.add_argument("--radius", type=float, default=0.3, help="radius of robot")
  args = parser.parse_args()

  with open(args.map) as map_file:
    map = yaml.safe_load(map_file)

  if "roadmap" not in map:
    print("Not a roadmap file!")
    exit()
    
  with open(args.schedule) as states_file:
    schedule = yaml.safe_load(states_file)

  animation = Animation(map, schedule, args.radius)

  if args.video:
    animation.save(args.video, args.speed)
  else:
    animation.show()

if __name__ == "__main__":
  main()
  
