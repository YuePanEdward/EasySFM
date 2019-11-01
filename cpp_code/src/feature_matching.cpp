

void pairwise_match(img1,img2,feature_type,match_strategy,hessian_threshold=800,nn_ratio=0.7) {


    if feature_type == 'SURF':
        # Detect keypoints and extract the SURF descriptors
        surf = cv2.xfeatures2d.SURF_create(hessian_threshold)
        (kps1, descs1) = surf.detectAndCompute(img1, None)
        (kps2, descs2) = surf.detectAndCompute(img2, None)
    elif feature_type == 'SIFT':
        # Detect keypoints and extract the SIFT descriptors
        sift = cv2.xfeatures2d.SIFT_create(hessian_threshold)
        (kps1, descs1) = sift.detectAndCompute(img1, None)
        (kps2, descs2) = sift.detectAndCompute(img2, None)

    print('Extract',feature_type,'feature done')
    print('[Img 2] # kps: {}, descriptors: {}'.format(len(kps2), descs2.shape))
    print('[Img 1] # kps: {}, descriptors: {}'.format(len(kps1), descs1.shape))


    # Feature Matching
    if match_strategy == 'mutual_nn':
        # Method 1: Mutual nearest neighbor
        bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        matches = bf.match(descs1, descs2)
        matches= sorted(matches, key = lambda x:x.distance, reverse=False) # small to big
        matching_result = cv2.drawMatches(img1, kps1, img2, kps2, matches[:50:], None, flags=2)

    elif match_strategy == 'ratio_test':
        # Method 2: Nearest neighbor with ratio test
        bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
        matches_2nn = bf.knnMatch(descs1, descs2, k=2)
        # Apply ratio test
        matches = []
        for m, n in matches_2nn:
            if m.distance < nn_ratio * n.distance:
                matches.append([m])
        matching_result = cv2.drawMatchesKnn(img1, kps1, img2, kps2, matches[:50:], None, flags=2)

    print('Get',len(matches),'correspondences using',match_strategy,'matching strategy')

}