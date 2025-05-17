# Use the official Flutter image
#FROM ghcr.io/flutter/flutter:latest
FROM ghcr.io/cirruslabs/flutter:3.29.1

# Install additional dependencies if needed
RUN apt-get update && apt-get install -y \
    git unzip curl xz-utils libglu1-mesa


RUN apt update
RUN apt install clang -y
RUN apt install -y cmake
RUN apt install ninja-build -y
RUN apt install pkg-config -y
RUN apt install libgtk-3-dev -y

# Get dependencies
#RUN flutter pub get

# Expose necessary ports
EXPOSE 8080

# Default command (change if necessary)
CMD ["/bin/bash"]


#code /root/.pub-cache/hosted/pub.dev/flutter_swipable-1.2.1/lib/flutter_swipable.dart
#find: child: Stack(overflow: Overflow.visible, children: [
#replace: child: Stack(clipBehavior: Clip.none, children: [

#then:
    # flutter clean
    # flutter pub get
    # flutter run
    