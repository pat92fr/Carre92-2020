from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import TensorBoard, EarlyStopping, ModelCheckpoint, Callback

import matplotlib.pyplot as plt
import time

## FUNCTIONS ###################################################################

# updatable plot
class PlotLosses(Callback):
    def on_train_begin(self, logs={}):
        self.i = 0
        self.x = []
        self.losses = []
        self.val_losses = []
        self.fig = plt.figure()
        self.logs = []

    def on_epoch_end(self, epoch, logs={}):
        
        self.logs.append(logs)
        self.x.append(self.i)
        self.losses.append(logs.get('loss'))
        self.val_losses.append(logs.get('val_loss'))
        self.i += 1
        
        #clear_output(wait=True)
        plt.clf()
        plt.plot(self.x, self.losses, label="loss")
        plt.plot(self.x, self.val_losses, label="val_loss")
        plt.legend()
        plt.pause(0.001)
        #plt.show(block=False);

# history plot
plot_losses = PlotLosses()

width = 1280//8
height = 720//8

model = Sequential()
model.add( Conv2D(8, (3, 3), input_shape=(height, width, 3)) )
model.add( Activation('relu') )
model.add( MaxPooling2D(pool_size=(2, 2)) )

model.add( Conv2D(12, (3, 3)) )
model.add( Activation('relu') )
model.add( MaxPooling2D(pool_size=(2, 2)) )

model.add(Conv2D(16, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(24, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Flatten())  # this converts our 3D feature maps to 1D feature vectors
model.add(Dense(32))
model.add(Activation('relu'))
model.add(Dropout(0.5))
# output layer
model.add(Dense(4))
model.add(Activation('softmax'))

model.compile(loss='categorical_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])

model.summary()

batch_size = 32

# this is the augmentation configuration we will use for training
train_datagen = ImageDataGenerator(
        rescale=1./255,
        width_shift_range=0.1,
        height_shift_range=0.1,
        shear_range=0.1,
        zoom_range=0.1,
        horizontal_flip=True)

# this is the augmentation configuration we will use for testing:
# only rescaling
test_datagen = ImageDataGenerator(rescale=1./255)

# this is a generator that will read pictures found in
# subfolers of 'data/train', and indefinitely generate
# batches of augmented image data
train_generator = train_datagen.flow_from_directory(
        'data/training_set',  # this is the target directory
        target_size=(height, width),  # all images will be resized to 150x150
        batch_size=batch_size,
        class_mode='categorical')  # since we use binary_crossentropy loss, we need binary labels

# this is a similar generator, for validation data
validation_generator = test_datagen.flow_from_directory(
        'data/test_set',
        target_size=(height, width),
        batch_size=batch_size,
        class_mode='categorical')

history = model.fit_generator(
        train_generator,
        steps_per_epoch=2000 // batch_size,
        epochs=20,
        validation_data=validation_generator,
        validation_steps=800 // batch_size,
        callbacks=[plot_losses],
        verbose = 2) # one line per epoch
model.save('first_try.h5')  # always save your weights after training or during training

print("Done.")

## list all data in history
print(history.history.keys())

## history
print("Saving history to disk...")
plt.clf()
plt.plot(history.history['acc'], label='train')
plt.plot(history.history['val_acc'], label='test')
plt.yscale("log")
plt.ylabel('accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper right')
plt.savefig("history" + '/' + 'history' + '_' + time.asctime().replace(' ', '_').replace(':', '-') + '.png')
print("Done.")
