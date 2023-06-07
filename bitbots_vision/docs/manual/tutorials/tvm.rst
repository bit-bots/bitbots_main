TVM
===

We use `TVM <https://tvm.apache.org//>`_ to compile our neural networks to run on the GPU of the robot. 
The networks are compiled to Vulkan shaders which run on many different GPUs including non Nvidia GPUs that don't support CUDA.
This results in a significant speedup compared to running the networks on the CPU.

Installation on the robot
--------------------------

- Follow install/build `guide <https://tvm.apache.org/docs/install/from_source.html/>`_
    - Install Vulkan SDK with this `script <https://github.com/apache/tvm/blob/main/docker/install/ubuntu_install_vulkan.sh/>`_ in `docker/install` in the TVM repo
    - Change `build/config.cmake` to enable profiling, llvm and vulkan during TVM build
- Install helpfull Vulkan tools 

    .. code-block:: bash

        sudo apt install vulkan-tools

- Add user to group ``render`` and ``video``

    .. code-block:: bash

        sudo usermod -a -G render,video $USER

- Install tvm python directory with 

    .. code-block:: bash

        pip install ./python

- Install additional deps

    .. code-block:: bash

        pip install onnx onnxsim xgboost


How to optimize a model
------------------------

- Run the following commands if you are anoyed by deprication warnings

    .. code-block:: bash

        export PYTHONWARNINGS="ignore"

- Use tvmc (located in ``~/.local/bin``, therefore check if it is in the ``PATH``!) for all simple TVM related things. If it is not in the ``PATH`` you can add it with

        .. code-block:: bash

            export PATH=$PATH:~/.local/bin

    You can add this to your ``.bashrc`` (if you use bash) or ``.zshrc`` (if you use zsh) to make it permanent.
    
- Simplify your onnx model using

        .. code-block:: bash

            onnxsim <input_model> <output_model>

    This step is needed as onnx can be quite convoluted and TVM does not support all operations.

- Tune your simplified onnx model using

        .. code-block:: bash
    
            tvmc tune --target "vulkan" --target-vulkan-from_device 0 --enable-autoscheduler --output <optimization_run_name>.json --enable-autoscheduler --repeat 5 --number 50 <onnx_file>

  - Only do this on the "NUCs" because the model will be optimized for the specific hardware.
  - If you only optimize for CPU select target "llvm" and possibly narrow it down further by setting llvm related settings, but we focus Vulkan (GPU) optimization for the rest of this guide. 
  - Remember to replace the placeholders in the command.
  - Check with ``radeontop`` if the GPU is utilized.
  - The optimization might take hours or even days.

- Compile the model using the optimizations in the json file from the previous tuning. To do this run

        .. code-block:: bash
    
            tvmc compile --target "vulkan" --target-vulkan-from_device 0 --output <output_name>.tar --tuning-records <optimization_run_name>.json <onnx_file>

    - You also want to do this on the "NUC" because the model will be optimized for the specific hardware.
    - This should **not** take hours.
    - Remember to replace all placeholders in the command
    - You will hopefully end up with a ``.tar`` file containing the following items 
        - ``mod.so``
        - ``mod.params`` 
        - ``mod.json``

Run your compiled model
-----------------------

- To test the model run the following command

        .. code-block:: bash

            tvmc run <model_name>.tar --profile --print-time --device "vulkan" --repeat 100

    The command shows you a profiling of each layer. Check if they all run on ``vulkan0``. At the bottom a timing benchmark is printed.

Run the model using the Python API
----------------------------------

Extract the tar, using `tar -xf <your_file>`. 
The following code snippet shows how to run a `YOEO <https://github.com/bit-bots/YOEO/>`_ model using the Python API. 
Normally the ``bitbots_vision`` is used to run the compiled model.

    .. code-block:: python

        import numpy as np
        import tvm
        from tvm.contrib import graph_executor

        # Load model
        binary_lib = tvm.runtime.load_module("mod.so")
        loaded_params = bytearray(open("mod.params", "rb").read())
        loaded_json = open("mod.json").read()

        # Create model module
        module = graph_executor.create(loaded_json, binary_lib, tvm.vulkan(0)) # Replace with tvm.cpu(0) for CPU models
        module.load_params(loaded_params)

        # Create dummy data
        input_data = np.random.uniform(size=(1,3,  416, 416)).astype("float32")

        # Run the network
        module.set_input(InputLayer=input_data)
        module.run()
        yolo_detections, segmentation = module.get_output(0).numpy(), module.get_output(1).numpy()

        print(yolo_detections.shape, segmentation.shape)


Compile a YOEO model directly
-----------------------------

If you use YOEO you can export the model to onnx with the ``yoeo-to-onnx`` command. See ``yoeo-to-onnx --help`` for more information.
You can then use the onnx model to compile it with TVM using the ``yoeo-onnx-to-tvm`` command. See ``yoeo-onnx-to-tvm --help`` for more information.
By doing so you can skip most of the steps above that use the ``tvmc`` cli. 
