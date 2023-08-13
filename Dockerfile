FROM python:3.11.4

# Set working directory
WORKDIR /

# Copy all project files
COPY . .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Run
CMD ["python", "./main.py"]
