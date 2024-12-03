# The purpose of this script is to load a trained Capsule Network model, 
# and use it to perform inference on a sample. The script returns the 
# pose matrix for the sample. The intended use is to pass many images 
# through this script, to generate a new dataset. This new dataset has 
# the pose matrices as its input and the ground thruth camera position 
# as its labels.

import tensorflow.compat.v1 as tf
import model as f_model
from mnist \
  import mnist_record
from norb \
  import norb_record
import numpy as np
import em_model

# Only the FLAGS that seem relevant
FLAGS = tf.app.flags.FLAGS
tf.app.flags.DEFINE_string('summary_dir',
                           '/home/swaltmann/mirte_ws/src/mirte-ros-packages/mirte_efficient_capsnet/pose_verification/summary_dir',
                           'Summaries log directory')
tf.app.flags.DEFINE_integer('batch_size', 32, 'Batch size.')  # Set to 1 for 1 image :)
FLAGS = tf.app.flags.FLAGS
tf.app.flags.DEFINE_integer('num_prime_capsules', 32,
                            'Number of first layer capsules.')
tf.app.flags.DEFINE_float('learning_rate', 0.01, 'Initial learning rate')
tf.app.flags.DEFINE_integer('routing_iteration', 3,
                            'Number of iterations for softmax routing')
tf.app.flags.DEFINE_float(
    'routing_rate', 1,
    'ratio for combining routing logits and routing feedback')
tf.app.flags.DEFINE_float('decay_rate', 0.96, 'ratio for learning rate decay')
tf.app.flags.DEFINE_integer('decay_steps', 20000,
                            'number of steps for learning rate decay')
tf.app.flags.DEFINE_bool('normalize_kernels', False,
                         'Normalize the capsule weight kernels')
tf.app.flags.DEFINE_integer('num_second_atoms', 16,
                            'number of capsule atoms for the second layer')
tf.app.flags.DEFINE_integer('num_primary_atoms', 16,
                            'number of capsule atoms for the first layer')
tf.app.flags.DEFINE_integer('num_start_conv', 32,
                            'number of channels for the start layer')
tf.app.flags.DEFINE_integer('kernel_size', 5,
                            'kernel size for the start layer.')
tf.app.flags.DEFINE_integer(
    'routing_iteration_prime', 1,
    'number of routing iterations for primary capsules.')
tf.app.flags.DEFINE_integer('max_steps', 2000000,
                            'Number of steps to run trainer.')
tf.app.flags.DEFINE_string('data_dir', '/datasets/mnist/',
                           'Directory for storing input data')
tf.app.flags.DEFINE_bool('train', True, 'train or test.')
tf.app.flags.DEFINE_integer(
    'checkpoint_steps', 1500,
    'number of steps before saving a training checkpoint.')
tf.app.flags.DEFINE_bool('verbose_image', False, 'whether to show images.')
tf.app.flags.DEFINE_bool('multi', True,
                         'whether to use multiple digit dataset.')
tf.app.flags.DEFINE_bool('eval_once', False,
                         'whether to evaluate once on the ckpnt file.')
tf.app.flags.DEFINE_integer('eval_size', 128,
                            'number of examples to evaluate.')
tf.app.flags.DEFINE_string(
    'ckpnt',
    '/home/swaltmann/mirte_ws/src/mirte-ros-packages/mirte_efficient_capsnet/pose_verification/capsule_em/model.ckpt-1',
    'The checkpoint to load and evaluate once.')
tf.app.flags.DEFINE_integer('keep_ckpt', 5, 'number of examples to evaluate.')
tf.app.flags.DEFINE_bool(
    'clip_lr', False, 'whether to clip learning rate to not go bellow 1e-5.')
tf.app.flags.DEFINE_integer('stride_1', 2,
                            'stride for the first convolutinal layer.')
tf.app.flags.DEFINE_integer('kernel_2', 9,
                            'kernel size for the secon convolutinal layer.')
tf.app.flags.DEFINE_integer('stride_2', 2,
                            'stride for the second convolutinal layer.')
tf.app.flags.DEFINE_string('padding', 'VALID',
                           'the padding method for conv layers.')
tf.app.flags.DEFINE_integer('extra_caps', 2, 'number of extra conv capsules.')
tf.app.flags.DEFINE_string('caps_dims', '32,32',
                           'output dim for extra conv capsules.')
tf.app.flags.DEFINE_string('caps_strides', '2,1',
                           'stride for extra conv capsules.')
tf.app.flags.DEFINE_string('caps_kernels', '3,3',
                           'kernel size for extra conv capsuls.')
tf.app.flags.DEFINE_integer('extra_conv', 0, 'number of extra conv layers.')

tf.app.flags.DEFINE_string('conv_dims', '', 'output dim for extra conv layers.')
tf.app.flags.DEFINE_string('conv_strides', '', 'stride for extra conv layers.')
tf.app.flags.DEFINE_string('conv_kernels', '',
                           'kernel size for extra conv layers.')
tf.app.flags.DEFINE_bool('leaky', False, 'Use leaky routing.')
tf.app.flags.DEFINE_bool('fast', False, 'Use the new faster implementation.')
tf.app.flags.DEFINE_bool('cpu_way', False,
                         'If set, use NHWC ordering instead of NCHW.')
tf.app.flags.DEFINE_bool('jit_scopes', False,
                         'Use xla jit_scopes to compile. Not supported.')
tf.app.flags.DEFINE_bool('staircase', False, 'Use staircase decay.')
tf.app.flags.DEFINE_integer('num_gpus', 1, 'number of gpus to train.')
tf.app.flags.DEFINE_bool('adam', True, 'Use Adam optimizer.')
tf.app.flags.DEFINE_bool('pooling', False, 'Pooling after convolution.')
tf.app.flags.DEFINE_bool('use_caps', True, 'Use capsule layers.')
tf.app.flags.DEFINE_integer(
    'extra_fc', 512, 'number of units in the extra fc layer in no caps mode.')
tf.app.flags.DEFINE_bool('dropout', False, 'Dropout before last layer.')
tf.app.flags.DEFINE_bool('tweak', False, 'During eval recons from tweaked rep.')
tf.app.flags.DEFINE_bool('softmax', False, 'softmax loss in no caps.')
tf.app.flags.DEFINE_bool('c_dropout', False, 'dropout after conv capsules.')
tf.app.flags.DEFINE_bool(
    'distort', True,
    'distort mnist images by cropping to 24 * 24 and rotating by 15 degrees.')
tf.app.flags.DEFINE_bool('restart', False, 'Clean train checkpoints.')
tf.app.flags.DEFINE_bool('use_em', True,
                         'If set use em capsules with em routing.')
tf.app.flags.DEFINE_float('final_beta', 0.01, 'Temperature at the sigmoid.')
tf.app.flags.DEFINE_bool('eval_ensemble', False, 'eval over aggregated logits.')
tf.app.flags.DEFINE_string('part1', 'ok', 'ok')
tf.app.flags.DEFINE_string('part2', 'ok', 'ok')
tf.app.flags.DEFINE_bool('reduce_mean', False,
                         'If set normalize mean of each image.')
tf.app.flags.DEFINE_float('loss_rate', 1.0,
                          'classification to regularization rate.')
tf.app.flags.DEFINE_integer('norb_pixel', 48, 'Batch size.')
tf.app.flags.DEFINE_bool('patching', True, 'If set use patching for eval.')

tf.app.flags.DEFINE_string('data_set', 'norb', 'the data set to use.')
tf.app.flags.DEFINE_string('cifar_data_dir', '/tmp/cifar10_data',
                           """Path to the CIFAR-10 data directory.""")
tf.app.flags.DEFINE_string('norb_data_dir', '/home/swaltmann/mirte_ws/src/mirte-ros-packages/mirte_efficient_capsnet/pose_verification/capsule_em/smallNORB',
                           """Path to the norb data directory.""")
tf.app.flags.DEFINE_string('affnist_data_dir', '/tmp/affnist_data',
                           """Path to the affnist data directory.""")



# Unchanged in case I want to try other datasets as well
def get_features(train, total_batch):
	"""Return batched inputs."""
	print(FLAGS.data_set)
	batch_size = total_batch // max(1, FLAGS.num_gpus)
	split = 'train' if train else 'test'
	features = []
	for i in range(FLAGS.num_gpus):
		with tf.device('/cpu:0'):
			with tf.name_scope('input_tower_%d' % (i)):
				if FLAGS.data_set == 'norb':
					features += [
              			norb_record.inputs(
							train_dir=FLAGS.norb_data_dir,
							batch_size=batch_size,
							split=split,
							multi=FLAGS.multi,
							image_pixel=FLAGS.norb_pixel,
							distort=FLAGS.distort,
							patching=FLAGS.patching,
              			)
					]
				elif FLAGS.data_set == 'affnist':
					features += [
						mnist_record.inputs(
							train_dir=FLAGS.affnist_data_dir,
							batch_size=batch_size,
							split=split,
							multi=FLAGS.multi,
							shift=0,
							height=40,
							train_file='test.tfrecords')
        			]
				elif FLAGS.data_set == 'expanded_mnist':
					features += [
              			mnist_record.inputs(
                  			train_dir=FLAGS.data_dir,
                  			batch_size=batch_size,
                  			split=split,
                  			multi=FLAGS.multi,
                  			height=40,
                  			train_file='train_6shifted_6padded_mnist.tfrecords',
                  			shift=6)
          			]
				else:
					if train and not FLAGS.distort:
						shift = 2
					else:
						shift = 0
					features += [
              			mnist_record.inputs(
                  			train_dir=FLAGS.data_dir,
                  			batch_size=batch_size,
                  			split=split,
                  			multi=FLAGS.multi,
                  			shift=shift,
                  			distort=FLAGS.distort)
					]				
	print("\n\nPast all the warnings from get_features()\n\n")
	return features


def infer_sample():
    with tf.Graph().as_default():
        # Input images and labels.
        features = get_features(True, FLAGS.batch_size)  # Load features
        print("Feature details:")
        for key, value in features[0].items():
            print(key, value)

        # Define inference model
        inference = em_model.inference
        result = {}
        y, result['recons_1'], result['recons_2'], result['mid_act'] = inference(features[0])
        result['logits'] = y

        print("\n\n RESULTS: \n")

        # Create and run a TensorFlow session
        with tf.Session() as sess:
            # If using pre-trained weights, restore them here
            # Example: saver.restore(sess, 'path_to_checkpoint') if needed
            
            # Initialize all variables (important if not restoring from a checkpoint)
            sess.run(tf.global_variables_initializer())
            
            # Evaluate and print results
            for key, value in result.items():
                if value is not None:  # Ensure it is not None before running
                    try:
                        # Evaluate the tensor
                        evaluated_value = sess.run(value)
                        print(f"Value in {key} are:")
                        print(evaluated_value)
                    except Exception as e:
                        print(f"Could not evaluate {key}: {e}")
                else:
                    print(f"Value in {key} are:")
                    print(value)  # Prints None
    print("DONE inferring")


def eval_once(ckpnt):
    """Evaluate on one checkpoint once."""
    ptches = np.zeros((14, 14, 32, 32))
    for i in range(14):
        for j in range(14):
            ind_x = i * 2
            ind_y = j * 2
            for k in range(5):
                for h in range(5):
                    ptches[i, j, ind_x + k, ind_y + h] = 1
    ptches = np.reshape(ptches, (14 * 14, 32, 32))

    with tf.Graph().as_default():
        features = get_features(False, 1)[0]
        if FLAGS.patching:
            features['images'] = features['cc_images']
            features['recons_label'] = features['cc_recons_label']
            features['labels'] = features['cc_labels']
        model = f_model.multi_gpu_model
        result = model([features])
        # merged = result['summary']
        correct_prediction_sum = result['correct']
        # almost_correct_sum = result['almost']
        # mid_act = result['mid_act']
        logits = result['logits']

        saver = tf.train.Saver()
        test_writer = tf.summary.FileWriter(FLAGS.summary_dir + '/test_once')
        config = tf.ConfigProto(allow_soft_placement=True)
        config.gpu_options.per_process_gpu_memory_fraction = 0.3
        sess = tf.Session(config=config)
        # saver.restore(sess, tf.train.latest_checkpoint(FLAGS.ckpnt))
        saver.restore(sess, ckpnt)
        coord = tf.train.Coordinator()
        threads = tf.train.start_queue_runners(sess=sess, coord=coord)
        i = 0
        print("Entering the try thing now")
        try:
            total_tp = 0
            for i in range(FLAGS.eval_size):
                #, g_ac, ac
                lb, tp, lg = sess.run([
                    features['recons_label'],
                    correct_prediction_sum,
                    logits,
                ])
                if FLAGS.patching:
                    batched_lg = np.sum(lg / np.sum(lg, axis=1, keepdims=True), axis=0)
                    batch_pred = np.argmax(batched_lg)
                    tp = np.equal(batch_pred, lb[0])

                total_tp += tp
            total_false = FLAGS.eval_size - total_tp
            print('false:{}, true:{}'.format(total_false, total_tp))
            # summary_tp = tf.Summary.FromString(summary_j)
            # summary_tp.value.add(tag='correct_prediction', simple_value=total_tp)
            # summary_tp.value.add(tag='wrong_prediction', simple_value=total_false)
            # summary_tp.value.add(
            #     tag='almost_wrong_prediction', simple_value=total_almost_false)
            # test_writer.add_summary(summary_tp, i + 1)
        except tf.errors.OutOfRangeError:
            print('Done eval for %d steps.' % i)
        finally:
            # When done, ask the threads to stop.
            coord.request_stop()
        # Wait for threads to finish.
        coord.join(threads)
        sess.close()
        test_writer.close()

def main(_):

    if tf.gfile.Exists(FLAGS.summary_dir + '/test_once'):
        tf.gfile.DeleteRecursively(FLAGS.summary_dir + '/test_once')
    tf.gfile.MakeDirs(FLAGS.summary_dir + '/test_once')
    eval_once(FLAGS.ckpnt)


if __name__ == '__main__':
	tf.app.run()
