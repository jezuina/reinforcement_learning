import numpy as np
import random
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, Conv2D
from keras.optimizers import Adam, RMSprop
from keras.utils import plot_model

from collections import deque

class OTNeuralNetwork:
    def __init__(self):
	self.state_dimension=4
	self.action_space=4
        self.memory  = deque(maxlen=2000)
        
        self.gamma = 0.85
        self.epsilon = 10.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.08
        self.tau = .125

        self.model        = self.create_model()
        self.target_model = self.create_model()

    def create_model(self):
        try:	
	    model   = Sequential()
            model.add(Dense(24, input_shape=(2,2), activation="relu"))
            model.add(Dense(48, activation="relu"))
	    model.add(Dense(48, activation="relu"))
            model.add(Dense(24, activation="relu"))
            model.add(Flatten())
            model.add(Dense(4))
            model.compile(loss="mean_squared_error",
                  optimizer=Adam(lr=self.learning_rate))
            #model.load_weights("weights")
	    print "krijohen modelet"
            return model
        except Exception as e:
	    print e
    def act(self, state):
        state=np.array(state)
	state=state.reshape(1,2,2)
	#print "state"
	#print state
        self.epsilon *= self.epsilon_decay
	print "epsilon"
	print self.epsilon
        self.epsilon = max(self.epsilon_min, self.epsilon)
	#print "epsilon"
	#print self.epsilon
        if np.random.random() < self.epsilon:	    
	    print "random"
	    randomn = random.randint(0,3)
	    #print randomn
            return randomn
	#print "argmax"
        maxn = np.argmax(self.model.predict(state))
	#print maxn
        return maxn

    def remember(self, state, action, reward, newState, done):
        self.memory.append([state,action,reward, newState, done])
	#for a in self.memory:
            #print a
	
    def replay(self):
        batch_size = 32
	#print "memory size "
	#print len(self.memory)
        if len(self.memory) < batch_size: 
            return

        samples = random.sample(self.memory, batch_size)
	#print "samples"
	#print samples
        for sample in samples:
            state, action, reward, new_state, done = sample
	    #print "sample data"
	    #print "statee"
	    #print state
            state=np.array(state)
	    state=state.reshape(1,2,2)
	    new_state=np.array(new_state)
	    new_state=new_state.reshape(1,2,2)
	    #print state
	    #print "new state"
	    #print new_state
            target = self.target_model.predict(state)
	    #print "target"
	    #print target
            if done:
		#print "done"
                target[0][action] = reward
            else:
		try:
		    #print "not done"
	            Q_future = max(self.target_model.predict(new_state)[0])
		    #print "action"
		    #print action
	            target[0][action] = reward + Q_future * self.gamma
		except Exception as e:
	            print e
            self.model.fit(state, target, epochs=1, verbose=0)
	plot_model(self.model, to_file='model.png')	
	#print "weights"
	#print self.model.get_weights()

    def target_train(self):
        weights = self.model.get_weights()
        target_weights = self.target_model.get_weights()
        for i in range(len(target_weights)):
            #target_weights[i] = weights[i] * self.tau + target_weights[i] * (1 - self.tau)
	    target_weights[i] = weights[i] + target_weights[i]
        self.target_model.set_weights(target_weights)

    def save_model(self, fn):
	#print "weights"
	#print fn
	# serialize model to JSON
	#model_json = self.model.to_json()
	#with open("model.json", "w") as json_file:
    	     #json_file.write(model_json)
	# serialize weights to HDF5
	self.model.save_weights("model.h5")
	print("Saved model to disk")
        #self.model.save_weights("/home/jkoroveshi/weights.model")



    def writeToFile(self,episodeLength):
	print "write ti file"
        try:
	    episodeLength=np.array(episodeLength)
	    print episodeLength
            np.savetxt('episodeLength.out', episodeLength, fmt="%d")   # X is an array
	except Exception as e:
   	    print e

