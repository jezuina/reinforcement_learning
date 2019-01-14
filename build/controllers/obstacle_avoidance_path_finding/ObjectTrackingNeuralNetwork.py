import numpy as np
import random
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.optimizers import Adam
from keras.utils import plot_model

from collections import deque

class OTNeuralNetwork:
    def __init__(self):
	self.state_dimension=4
	self.action_space=4
        self.memory  = deque(maxlen=2000)
        
        self.gamma = 0.85
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.005
        self.tau = .125

        self.model        = self.create_model()
        self.target_model = self.create_model()

    def create_model(self):
        model   = Sequential()
        state_shape  = [self.state_dimension,self.action_space]
        model.add(Dense(24, input_dim=self.state_dimension, activation="relu"))
        model.add(Dense(48, activation="relu"))
        model.add(Dense(24, activation="relu"))
        model.add(Dense(state_shape[1]))
        model.compile(loss="mean_squared_error",
            optimizer=Adam(lr=self.learning_rate))
	print "krijohen modelet"
        return model

    def act(self, state):
        state=np.array(state)
	state=state.reshape(1,self.state_dimension)
	print "state"
	print state
        self.epsilon *= self.epsilon_decay
	print "epsilon"
	print self.epsilon
        self.epsilon = max(self.epsilon_min, self.epsilon)
	print "epsilon"
	print self.epsilon
        if np.random.random() < self.epsilon:	    
	    print "random"
            return random.randint(0,4)
	print "argmax"
        return np.argmax(self.model.predict(state))

    def remember(self, state, action, reward, newState, done):
        self.memory.append([state,action,reward, newState, done])
	for a in self.memory:
            print a
	
    def replay(self):
        batch_size = 32
	print "memory size "
	print len(self.memory)
        if len(self.memory) < batch_size: 
            return

        samples = random.sample(self.memory, batch_size)
	print "samples"
	print samples
        for sample in samples:
            state, action, reward, new_state, done = sample
	    print "sample data"
	    print "statee"
	    print state
            state=np.array(state)
	    state=state.reshape(1,self.state_dimension)
	    new_state=np.array(new_state)
	    new_state=new_state.reshape(1,self.state_dimension)
	    print state
            target = self.target_model.predict(state)
	    print "target"
	    print target
            if done:
		print "done"
                target[0][action] = reward
            else:
		print "not done"
                Q_future = max(self.target_model.predict(new_state)[0])
                target[0][action] = reward + Q_future * self.gamma
            self.model.fit(state, target, epochs=1, verbose=0)
	plot_model(self.model, to_file='model.png')	
	#print "weights"
	#print self.model.get_weights()

    def target_train(self):
        weights = self.model.get_weights()
        target_weights = self.target_model.get_weights()
        for i in range(len(target_weights)):
            target_weights[i] = weights[i] * self.tau + target_weights[i] * (1 - self.tau)
        self.target_model.set_weights(target_weights)

    def save_model(self, fn):
	print "filename"
	print fn
        self.model.save(fn)
