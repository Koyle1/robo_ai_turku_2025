import cv2
import numpy as np
import datetime

def process_image():
    # TODO: Load an image from file
    # Hint: Use cv2.imread()
    image_path = "images/drone_image.jpg"  
    img = cv2.imread(image_path)  
    
    if img is None:
        print("Error: Could not load image")
        return
    
    print(f"Image shape: {img.shape}")
    print(f"Image dtype: {img.dtype}")
    
    # TODO: Draw a rectangle on the image
    cv2.rectangle(img, (10, 10), (50, 50), (0, 255, 0), 5)
    
    # TODO: Draw a circle on the image
    cv2.circle(img, (100, 100), 30, (255, 0, 0), 5)
    
    # TODO: Draw a filled polygon on the image
    cv2.fillPoly(img, [np.array([[200, 200], [250, 250], [200, 300]])], (0, 0, 255))
    
    # TODO: Add text to the image
    # Hint: cv2.putText(img, text, position, font, scale, color, thickness)
    cv2.putText(img, "Felix Coy & Peter Kruse & Michael Krone", (150, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    now = datetime.datetime.now()
    cv2.putText(img, now.strftime("%Y-%m-%d %H:%M:%S"), (150, 200), cv2.FONT_HERSHEY_DUPLEX, 1, (122, 123, 0), 10)
    
    # TODO: Display the image
    # Hint: cv2.imshow() and cv2.waitKey()
    cv2.imshow("Processed Image", img)
    cv2.waitKey(0)
    
    # TODO: Save the processed image
    # Hint: cv2.imwrite()
    output_path = "processed_images/task1_processed.png"  
    cv2.imwrite(output_path, img)
    print(f"Processed image saved to {output_path}")

if __name__ == "__main__":
    process_image()