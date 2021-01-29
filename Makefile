TEST_PATH=./

help:
	@echo "    main"
	@echo "        Run main python file"
	@echo "    test"
	@echo "        Run unit tests"




.EXPORT_ALL_VARIABLES:

ROOT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

main:   
	python3 pozyx_publisher.py




















