# at the top of your Makefile
GIT_ACCESS_TOKEN ?=          # default is empty
GIT_ACCESS_TOKEN := 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

.PHONY: build
build:
	docker build \
	  --build-arg GIT_ACCESS_TOKEN=${GIT_ACCESS_TOKEN} \
	  -t dynus .

.PHONY: build-no-cache
build-no-cache:
	docker build \
	--build-arg GIT_ACCESS_TOKEN=${GIT_ACCESS_TOKEN} \
	--build-arg DUMMY=`date +%s` -t dynus .

run:
	xhost + && docker run  -it \
				--gpus all \
				--env="NVIDIA_DRIVER_CAPABILITIES=all" \
				--privileged \
				-e DISPLAY=${DISPLAY} \
				--network=host \
				--env="DISPLAY" \
				--env="QT_X11_NO_MITSHM=1" \
				--rm dynus