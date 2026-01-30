FROM krinkin/gtl-cy-2026-opencv

SHELL ["/bin/bash", "-c"]

WORKDIR /workspace

COPY requirements-apt.txt .
COPY requirements-python.txt .

RUN set -e; \
    if [ -s requirements-python.txt ]; then \
        pip install -r requirements-python.txt; \
    fi
