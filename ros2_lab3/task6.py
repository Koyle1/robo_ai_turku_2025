#!/usr/bin/env python3
"""
Task 4: Convolution Blur - Student Template
Apply a blurring filter to an image using manual convolution
(without cv2.filter2D).
"""

import cv2
import numpy as np

def convolve(image, kernel):
    """Perform convolution on a grayscale image with a given kernel."""
    # Get image and kernel dimensions
    img_h, img_w = image.shape
    k_h, k_w = kernel.shape

    # Pad the image with zeros around the border
    pad_h = k_h // 2
    pad_w = k_w // 2
    padded = np.pad(image, ((pad_h, pad_h), (pad_w, pad_w)), mode='constant')

    # Create an empty output image
    output = np.zeros_like(image, dtype=np.float32)

    # Loop over each pixel
    for y in range(img_h):
        for x in range(img_w):
            # Extract the region of interest
            region = padded[y:y + k_h, x:x + k_w]
            # Multiply by kernel and sum
            output[y, x] = np.sum(region * kernel)

    # Clip values to 0–255
    output = np.clip(output, 0, 255).astype(np.uint8)

    return output  

def main():
    image_path = "images/flower.png"
    # Read image in grayscale
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError("Image file not found. Place 'flower.png' in the same folder.")
    
    # Define a simple 5x5 averaging kernel (all ones / 25)
    kernel = np.ones((5, 5), dtype=np.float32) / 25.0

    # Define Sobel operators
    sobel_x = np.array([[-1, 0, 1],
                        [-2, 0, 2],
                        [-1, 0, 1]], dtype=np.float32)
    
    sobel_y = np.array([[1, 2, 1],
                        [0, 0, 0],
                        [-1, -2, -1]], dtype=np.float32)

    # Apply convolution (blur)
    blurred = convolve(image, kernel)

    # Apply convolution with Sobel filters
    grad_x = convolve(image, sobel_x)
    grad_y = convolve(image, sobel_y)

    # Compute the edge magnitude
    edges = np.sqrt(grad_x.astype(np.float32)**2 + grad_y.astype(np.float32)**2)
    edges = np.clip(edges, 0, 255).astype(np.uint8)
    edges = cv2.normalize(edges, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Display results
    cv2.imshow("Original", image)
    cv2.imshow("Blurred", blurred)
    cv2.imshow("Sobel X", grad_x)
    cv2.imshow("Sobel Y", grad_y)
    cv2.imshow("Edges", edges)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
