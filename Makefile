module.tar.gz: poetry.lock pyproject.toml requirements.txt .env *.sh src/*.py
	tar czf module.tar.gz $^
