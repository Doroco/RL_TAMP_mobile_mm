#!/usr/bin/env python

import matplotlib as mpl
import matplotlib.pyplot as plt

import tensorflow as tf
from keras.layers import *
from keras.callbacks import ModelCheckpoint
from keras.layers.merge import Concatenate
from keras.models import Sequential, Model
from keras.optimizers import *
from keras import backend as K
from keras import initializers

import time
import sys
import numpy as np
import os
import random

def G_loss(dummy, pred):
    return -K.mean(pred, axis=-1)  # try to maximize the value of pred


def noise(n, z_size):
    return np.random.normal(size=(n, z_size)).astype('float32')


def tile(x):
    reps = [1, 1, 32]
    return K.tile(x, reps)
    
class DDPG:
    def __init__(self, sess, dim_action, dim_state, tau, save_folder, explr_const, visualize=False):
        self.opt_G = tf.keras.optimizers.Adam(lr=1e-4, beta_1=0.5)
        self.opt_D = tf.keras.optimizers.Adam(lr=1e-3, beta_1=0.5)
        self.initializer = initializers.glorot_normal()
        self.sess = sess
        self.dim_action = dim_action
        self.dim_state = dim_state
        self.v = visualize
        self.tau = tau
        self.n_weight_updates = 0
        self.save_folder = save_folder
        self.explr_const = explr_const

        # define inputs
        self.x_input = Input(shape=(dim_action,), name='x', dtype='float32')  # action
        self.w_input = Input(shape=(dim_state,), name='w', dtype='float32')   # collision vector
        self.a_gen, self.disc, self.DG, = self.createGAN()

    def createGAN(self):
        disc = self.createDisc()
        a_gen, a_gen_output = self.createGen()
        for l in disc.layers:
            l.trainable = False
        DG_output = disc([a_gen_output, self.w_input])
        DG = Model(input=[self.w_input], output=[DG_output])
        DG.compile(loss={'disc_output': G_loss, },
                   optimizer=self.opt_G,
                   metrics=[])
        return a_gen, disc, DG

    def saveWeights(self, init=True, additional_name=''):
        self.a_gen.save_weights(self.save_folder + '/a_gen' + additional_name + '.h5')
        self.disc.save_weights(self.save_folder + '/disc' + additional_name + '.h5')

    def load_offline_weights(self, weight_f):
        self.a_gen.load_weights(self.save_folder + weight_f)

    def load_weights(self):
        best_rwd = -np.inf
        for weightf in os.listdir(self.save_folder):
            if weightf.find('a_gen') == -1: continue
            try:
                rwd = float(weightf.split('_')[-1][0:-3])
            except ValueError:
                continue
            if rwd > best_rwd:
                best_rwd = rwd
                best_weight = weightf
        print "Using initial weight ", best_weight
        self.a_gen.load_weights(self.save_folder + '/' + best_weight)

    def resetWeights(self, init=True):
        if init:
            self.a_gen.load_weights('a_gen_init.h5')
            self.disc.load_weights('disc_init.h5')
        else:
            self.a_gen.load_weights(self.save_folder + '/a_gen.h5')
            self.disc.load_weights(self.save_folder + '/disc.h5')

    def createGen(self):
        init_ = self.initializer
        dropout_rate = 0.25
        dense_num = 64
        n_filters = 64

        # K_H = self.k_input
        H = Dense(dense_num, activation='relu')(self.w_input)
        H = Dense(dense_num, activation='relu')(H)
        a_gen_output = Dense(self.dim_action,
                             activation='linear',
                             init=init_,
                             name='a_gen_output')(H)
        a_gen = Model(input=[self.w_input], output=a_gen_output)
        return a_gen, a_gen_output

    def createDisc(self):
        init_ = self.initializer
        dropout_rate = 0.25
        dense_num = 64

        # K_H = self.k_input
        XK_H = Concatenate(axis=-1)([self.x_input, self.w_input])

        H = Dense(dense_num, activation='relu')(XK_H)
        H = Dense(dense_num, activation='relu')(H)

        disc_output = Dense(1, activation='linear', init=init_)(H)
        disc = Model(input=[self.x_input, self.w_input],
                     output=disc_output,
                     name='disc_output')
        disc.compile(loss='mse', optimizer=self.opt_D)
        return disc

    def predict(self, x, n_samples=1):
        if n_samples == 1:
            n = n_samples
            d = self.dim_action
            pred = self.a_gen.predict(x)
            noise = self.explr_const * np.random.randn(n, d)
            return pred + noise
        else:
            n = n_samples
            d = self.dim_action
            pred = self.a_gen.predict(np.tile(x, (n, 1, 1)))
            noise = self.explr_const * np.random.randn(n, d)
            return pred + noise

    def soft_update(self, network, before, after):
        new_weights = network.get_weights()
        for i in range(len(before)):
            new_weights[i] = (1 - self.tau) * before[i] + (self.tau) * after[i]
        network.set_weights(new_weights)

    def update_disc(self, batch_x, batch_w, batch_targets, batch_size):
        before = self.disc.get_weights()

        self.disc.fit({'x': batch_x, 'w': batch_w},
                      batch_targets,
                      validation_split=0.1,
                      batch_size=batch_size,
                      epochs=1,
                      verbose=False)
        after = self.disc.get_weights()
        self.soft_update(self.disc, before, after)

    def update_pi(self, s_batch, batch_size):
        # maximizes Q( pi(s_batch ) )
        y_labels = np.ones((len(s_batch),))  # dummy variable
        before = self.a_gen.get_weights()

        self.DG.fit({'w': s_batch},
                    {'disc_output': y_labels, 'a_gen_output': y_labels},
                    validation_split=0.1,
                    batch_size=batch_size,
                    epochs=1,
                    verbose=False)
        after = self.a_gen.get_weights()  # verfied that weights of disc does not change
        self.soft_update(self.a_gen, before, after)

    def augment_dataset(self, traj_list, states, actions, rewards, sprimes):
        new_s, new_a, new_r, new_sprime, new_sumR, _, new_traj_lengths = format_RL_data(traj_list)
        new_a = new_a
        new_data_obtained = len(new_s) > 0

        if new_data_obtained:
            if states is not None:
                n_new = len(new_s)
                n_dim_state = states.shape[1]
                states = np.r_[states, new_s.reshape((n_new, n_dim_state))]
                actions = np.r_[actions, new_a]
                rewards = np.r_[rewards, new_r]
                sprimes = np.r_[sprimes, new_sprime.reshape((n_new, n_dim_state))]
            else:
                states = new_s
                actions = new_a
                rewards = new_r
                sprimes = new_sprime
        else:
            pass

        if states is not None:
            terminal_state_idxs = np.where(np.sum(np.sum(sprimes, axis=-1), axis=-1) == 0)[0]
            nonterminal_mask = np.ones((sprimes.shape[0], 1))
            nonterminal_mask[terminal_state_idxs, :] = 0
        else:
            nonterminal_mask = None

        return states, actions, rewards, sprimes, nonterminal_mask, new_data_obtained
