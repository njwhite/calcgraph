#!/usr/bin/env python

import pandas as pd
import numpy as np
import time
import os
import random
import timeit
import socket
import subprocess

from manim import *

TIME = 1 / 120
MAXDIM = 10

class FlockingOutput(Scene):
    def _run_simulation(self, spawn=True, run_for_s=5):
        if spawn:
            p = subprocess.Popen(
                [os.path.join(os.getcwd(), 'example')], 
                shell=False)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

        t0 = timeit.default_timer()
        spawned = 0
        while timeit.default_timer() - t0 < run_for_s / 2:
            time.sleep(random.random())
            sock.sendto(
                bytes(
                    f"{spawned} "
                    f"{random.random() * MAXDIM} "
                    f"{random.random() * MAXDIM} "
                    f"{random.random()} "
                    f"{random.random()}", 
                    "utf-8"),
                ('127.0.0.1', 8080))
            spawned += 1

        time.sleep(run_for_s / 2) # run without new dots

        if spawn:
            p.terminate()
            p.wait()

    def construct(self):
        self.add(Rectangle(
            width=MAXDIM, 
            height=MAXDIM))

        self._run_simulation(spawn=True, run_for_s=5)

        outputs = pd.read_csv(
            'output.csv',
            sep='\t',
            low_memory=False,
            memory_map=True)
        outputs['action'] = 'mov'
        outputs.loc[
            outputs.drop_duplicates(subset=['label'], keep='first').index,
            'action'] = 'new'

        # wrap to grid here
        outputs['x'] = (outputs.x % MAXDIM) - (MAXDIM/2)
        outputs['y'] = (outputs.y % MAXDIM) - (MAXDIM/2)

        dots = {}
        for _, row in outputs[outputs.action == 'new'].iterrows():
            dot = Dot(point=(row.x, row.y, 0))
            dot.generate_target()
            self.add(dot)
            dot.scale(0)
            dots[row.label] = dot 

        for period, v in outputs.groupby('period'):
            animations = []
            for _, row in v.iterrows():
                dot = dots[row.label]
                if row.action == 'new':
                    dot.scale(1)
                dot.target.move_to((row.x, row.y, 0))
                animations.append(MoveToTarget(dot, rate_func=linear))
            if animations:
                self.play(*animations, run_time=TIME)
            else:
                self.wait(TIME)