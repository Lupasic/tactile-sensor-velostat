import logging
import yaml
import logging.config

with open('logging.yaml', 'r') as f:
    config = yaml.safe_load(f.read())
logging.config.dictConfig(config)
logger_name = "simpleExample"
logger = logging.getLogger(logger_name)
logger.info("Set logger %s",logger_name)