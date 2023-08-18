module.tar.gz: poetry.lock pyproject.toml requirements.txt run.sh src/*.py
	tar czf module.tar.gz $^
