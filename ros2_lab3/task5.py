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

    # Pad the image with zeros around the border (zero padding)
    pad_h = k_h // 2
    pad_w = k_w // 2
    padded = np.pad(image, ((pad_h, pad_h), (pad_w, pad_w)), mode='constant', constant_values=0)

    # Create an empty output image
    output = np.zeros_like(image, dtype=np.float32)

    # Loop over each pixel (y, x)
    for y in range(img_h):
        for x in range(img_w):
            # Extract the region of interest (ROI) from padded image
            roi = padded[y:y+k_h, x:x+k_w]
            # Multiply by kernel and sum up values
            output[y, x] = np.sum(roi * kernel)

    # Clip values to 0–255
    output = np.clip(output, 0, 255).astype(np.uint8)

    return output  

def create_gaussian_kernel(size, sigma=1.0):
    """Create a Gaussian kernel of given size and sigma."""
    kernel = np.zeros((size, size))
    center = size // 2
    
    # Calculate Gaussian values
    for i in range(size):
        for j in range(size):
            x = i - center
            y = j - center
            kernel[i, j] = np.exp(-(x**2 + y**2) / (2 * sigma**2))
    
    # Normalize the kernel so it sums to 1
    kernel = kernel / np.sum(kernel)
    return kernel

def main():
    image_path = "images/flower.png"
    # Read image in grayscale
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError("Image file not found. Place 'flower.png' in the same folder.")
    
    # Define a simple 5x5 averaging kernel (all ones / 25)
    box_kernel_5x5 = np.ones((5, 5)) / 25
    
    # Test different kernel sizes
    box_kernel_3x3 = np.ones((3, 3)) / 9
    box_kernel_7x7 = np.ones((7, 7)) / 49

    # Apply convolution with different kernel sizes
    blurred_3x3 = convolve(image, box_kernel_3x3)
    blurred_5x5 = convolve(image, box_kernel_5x5)
    blurred_7x7 = convolve(image, box_kernel_7x7)

    # Create Gaussian kernel and apply to image
    gaussian_kernel_5x5 = create_gaussian_kernel(5, sigma=1.0)
    gaussian_blur = convolve(image, gaussian_kernel_5x5)

    # Display results
    cv2.imshow("Original", image)
    cv2.imshow("Box Blur 3x3", blurred_3x3)
    cv2.imshow("Box Blur 5x5", blurred_5x5)
    cv2.imshow("Box Blur 7x7", blurred_7x7)
    cv2.imshow("Gaussian Blur 5x5", gaussian_blur)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Save results for comparison
    cv2.imwrite("processed_images/task5/box_blur_3x3.jpg", blurred_3x3)
    cv2.imwrite("processed_images/task5/box_blur_5x5.jpg", blurred_5x5)
    cv2.imwrite("processed_images/task5/box_blur_7x7.jpg", blurred_7x7)
    cv2.imwrite("processed_images/task5/gaussian_blur_5x5.jpg", gaussian_blur)

if __name__ == "__main__":
    main()