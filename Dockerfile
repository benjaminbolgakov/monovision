FROM ubuntu:20.04

# Disable interactive user-prompts during apt installations
ENV DEBIAN_FRONTEND=noninteractive

# Update and install essential system packages
RUN apt-get update && \
    apt-get -y --no-install-recommends install \
        python3 \
        python3-pip \
        nginx \
        git


RUN apt-get clean
RUN rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* # Remove temp files
    ## May be needed?
    # build-essential \
    # curl \
    # python3-dev \
    # python3-setuptools \
    # libffi-dev \
    # golang \

# Set working directory
WORKDIR /

# Copy all project files
COPY . .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

WORKDIR /

# Run
CMD ["ls"]
# CMD ["python3", "./main.py"]
